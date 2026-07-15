"""Bounded receive buffer for the bridge I/O-to-processing handoff."""

from __future__ import annotations

import threading
from collections import deque
from typing import Deque, Generic, Optional, Tuple, TypeVar


Item = TypeVar("Item")


class LatestFrameBuffer(Generic[Item]):
    """Preserve frame order while bounding latency under overload."""

    def __init__(self, capacity: int) -> None:
        if capacity < 1:
            raise ValueError("receive buffer capacity must be positive")
        self._capacity = capacity
        self._items: Deque[Item] = deque()
        self._condition = threading.Condition()
        self._closed = False

    def put(self, item: Item) -> None:
        """Append one item, discarding the oldest item when full."""
        with self._condition:
            if self._closed:
                raise RuntimeError("receive buffer is closed")
            if len(self._items) >= self._capacity:
                self._items.popleft()
            self._items.append(item)
            self._condition.notify()

    def get_batch(
            self, max_items: int, timeout: Optional[float] = None) -> Tuple[Item, ...]:
        """Return up to ``max_items`` in FIFO order, or empty after close/timeout."""
        if max_items < 1:
            raise ValueError("max_items must be positive")
        with self._condition:
            if not self._items and not self._closed:
                self._condition.wait(timeout)
            count = min(max_items, len(self._items))
            return tuple(self._items.popleft() for _ in range(count))

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    @property
    def closed_and_empty(self) -> bool:
        with self._condition:
            return self._closed and not self._items
