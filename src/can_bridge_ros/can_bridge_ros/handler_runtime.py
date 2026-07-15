"""Load and dispatch explicitly configured in-process frame handlers."""

from __future__ import annotations

import importlib
import json
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

from can_bridge_ros.handler_api import (
    FrameDisposition,
    FrameHandlerContext,
    FrameHandlerRegistration,
    FrameKey,
)


@dataclass
class _HandlerState:
    registration: FrameHandlerRegistration
    consecutive_failures: int = 0
    enabled: bool = True
    last_error_log: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)


def _load_factory(path: str):
    module_name, separator, attribute_name = path.partition(":")
    if not separator or not module_name or not attribute_name or ":" in attribute_name:
        raise ValueError(
            f"handler factory must use 'module:function', got {path!r}")
    module = importlib.import_module(module_name)
    factory = getattr(module, attribute_name, None)
    if not callable(factory):
        raise TypeError(f"handler factory {path!r} is not callable")
    return factory


def _parse_spec(raw_spec: str) -> Tuple[str, Mapping[str, Any]]:
    try:
        spec = json.loads(raw_spec)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid frame handler JSON: {exc.msg}") from exc
    if not isinstance(spec, dict):
        raise ValueError("frame handler spec must be a JSON object")
    unknown = set(spec) - {"factory", "config"}
    if unknown:
        raise ValueError(f"unknown frame handler fields: {sorted(unknown)}")
    factory_path = spec.get("factory")
    config = spec.get("config", {})
    if not isinstance(factory_path, str) or not factory_path.strip():
        raise ValueError("frame handler spec requires a non-empty factory")
    if not isinstance(config, dict):
        raise ValueError("frame handler config must be a JSON object")
    return factory_path, config


class FrameHandlerRuntime:
    """Own handler registration, O(1) dispatch, and lifecycle callbacks."""

    def __init__(
            self,
            registrations: Sequence[FrameHandlerRegistration],
            logger: Any,
            *,
            failure_limit: int = 3) -> None:
        if failure_limit < 1:
            raise ValueError("failure_limit must be positive")
        self._logger = logger
        self._failure_limit = failure_limit
        self._states = [_HandlerState(registration) for registration in registrations]
        self._handlers: Dict[Tuple[int, int], _HandlerState] = {}
        self._started = False
        self._active = False

        for state in self._states:
            for key in state.registration.keys:
                lookup_key = (key.channel_id, key.can_id)
                previous = self._handlers.get(lookup_key)
                if previous is not None:
                    raise ValueError(
                        f"frame handler key channel={key.channel_id}, "
                        f"CAN ID=0x{key.can_id:X} is claimed by both "
                        f"{previous.registration.name!r} and "
                        f"{state.registration.name!r}")
                self._handlers[lookup_key] = state

    @classmethod
    def from_specs(
            cls,
            specs: Iterable[str],
            context: FrameHandlerContext,
            **kwargs) -> "FrameHandlerRuntime":
        registrations: List[FrameHandlerRegistration] = []
        try:
            for raw_spec in specs:
                raw_spec = str(raw_spec).strip()
                if not raw_spec:
                    continue
                factory_path, config = _parse_spec(raw_spec)
                factory = _load_factory(factory_path)
                registration = factory(context, config)
                if not isinstance(registration, FrameHandlerRegistration):
                    raise TypeError(
                        f"handler factory {factory_path!r} did not return "
                        "FrameHandlerRegistration")
                registrations.append(registration)
            return cls(registrations, context.logger, **kwargs)
        except Exception:
            cls._destroy_nodes(registrations)
            raise

    @property
    def auxiliary_nodes(self) -> Tuple[Any, ...]:
        return tuple(
            node
            for state in self._states
            for node in state.registration.auxiliary_nodes)

    @property
    def registrations(self) -> Tuple[FrameHandlerRegistration, ...]:
        return tuple(state.registration for state in self._states)

    def start(self) -> None:
        if self._started:
            return
        started: List[_HandlerState] = []
        try:
            for state in self._states:
                if state.registration.start is not None:
                    state.registration.start()
                started.append(state)
        except Exception:
            for state in reversed(started):
                if state.registration.stop is not None:
                    try:
                        state.registration.stop()
                    except Exception as exc:  # noqa: BLE001
                        self._logger.error(
                            f"frame handler {state.registration.name!r} "
                            f"rollback failed: {exc}")
            raise
        self._started = True
        self._active = True

    def stop(self) -> None:
        if not self._started:
            return
        self._active = False
        self._started = False
        for state in reversed(self._states):
            if state.registration.stop is None:
                continue
            with state.lock:
                try:
                    state.registration.stop()
                except Exception as exc:  # noqa: BLE001
                    self._logger.error(
                        f"frame handler {state.registration.name!r} "
                        f"stop failed: {exc}")

    def destroy_auxiliary_nodes(self) -> None:
        self._destroy_nodes(self.registrations)

    def dispatch(self, channel_id: int, message: Any) -> FrameDisposition:
        if not self._active:
            return FrameDisposition.FORWARD
        can_id = int(message.arbitration_id)
        state = self._handlers.get((channel_id, can_id))
        if state is None or not state.enabled:
            return FrameDisposition.FORWARD

        with state.lock:
            if not self._active or not state.enabled:
                return FrameDisposition.FORWARD
            try:
                disposition = state.registration.callback(channel_id, message)
                if not isinstance(disposition, FrameDisposition):
                    raise TypeError(
                        f"callback returned {disposition!r}, "
                        "expected FrameDisposition")
            except Exception as exc:  # noqa: BLE001
                state.consecutive_failures += 1
                now = time.monotonic()
                if now - state.last_error_log >= 1.0:
                    state.last_error_log = now
                    self._logger.error(
                        f"frame handler {state.registration.name!r} failed "
                        f"({state.consecutive_failures}/{self._failure_limit}): {exc}")
                if state.consecutive_failures >= self._failure_limit:
                    state.enabled = False
                    self._logger.error(
                        f"frame handler {state.registration.name!r} disabled; "
                        "matching frames will use normal ROS routing")
                return FrameDisposition.FORWARD

            state.consecutive_failures = 0
            return disposition

    @staticmethod
    def _destroy_nodes(registrations: Sequence[FrameHandlerRegistration]) -> None:
        for registration in reversed(registrations):
            for node in reversed(registration.auxiliary_nodes):
                try:
                    node.destroy_node()
                except Exception:  # noqa: BLE001
                    pass