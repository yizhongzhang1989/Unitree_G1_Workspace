"""Exact BGR grayscale/saturation offload using the CUDA Driver API.

Single-caller worker object. Mapped pinned buffers avoid explicit device copies;
blocking synchronization yields the CPU instead of polling GPU completion.
"""

from __future__ import annotations

import ctypes as ct
from types import TracebackType
from typing import Any

import numpy as np


_PTX = b"""
.version 7.0
.target sm_70
.address_size 64
.visible .entry preprocess(
    .param .u64 source, .param .u64 destination,
    .param .u64 saturation_divisors, .param .u32 count)
{
    .reg .pred %p;
    .reg .b32 %r<18>;
    .reg .b64 %rd<10>;
    ld.param.u64 %rd1, [source];
    ld.param.u64 %rd2, [destination];
    ld.param.u64 %rd6, [saturation_divisors];
    ld.param.u32 %r4, [count];
    mov.u32 %r1, %ctaid.x;
    mov.u32 %r2, %ntid.x;
    mov.u32 %r3, %tid.x;
    mad.lo.u32 %r1, %r1, %r2, %r3;
    setp.ge.u32 %p, %r1, %r4;
    @%p bra DONE;
    mul.wide.u32 %rd3, %r1, 3;
    add.u64 %rd4, %rd1, %rd3;
    ld.global.u8 %r6, [%rd4];
    ld.global.u8 %r7, [%rd4+1];
    ld.global.u8 %r8, [%rd4+2];
    mul.lo.u32 %r9, %r6, 3735;
    mad.lo.u32 %r9, %r7, 19235, %r9;
    mad.lo.u32 %r9, %r8, 9798, %r9;
    add.u32 %r9, %r9, 16384;
    shr.u32 %r9, %r9, 15;
    cvt.u64.u32 %rd3, %r1;
    add.u64 %rd5, %rd2, %rd3;
    st.global.u8 [%rd5], %r9;
    max.u32 %r10, %r6, %r7;
    max.u32 %r10, %r10, %r8;
    min.u32 %r11, %r6, %r7;
    min.u32 %r11, %r11, %r8;
    sub.u32 %r12, %r10, %r11;
    mul.wide.u32 %rd7, %r10, 4;
    add.u64 %rd7, %rd6, %rd7;
    ld.global.u32 %r13, [%rd7];
    mad.lo.u32 %r14, %r12, %r13, 2048;
    shr.u32 %r14, %r14, 12;
    min.u32 %r15, %r10, 115;
    mad.lo.u32 %r16, %r14, %r15, 57;
    div.u32 %r16, %r16, 115;
    cvt.u64.u32 %rd8, %r4;
    add.u64 %rd5, %rd5, %rd8;
    st.global.u8 [%rd5], %r16;
DONE:
    ret;
}
"""


class CudaPreprocessor:
    def __init__(self) -> None:
        self._cuda = ct.CDLL('libcuda.so.1')
        pointer = ct.c_uint64
        signatures = {
            'cuGetErrorName': [ct.c_int, ct.POINTER(ct.c_char_p)],
            'cuInit': [ct.c_uint],
            'cuDeviceGet': [ct.POINTER(ct.c_int), ct.c_int],
            'cuCtxCreate_v2': [ct.POINTER(ct.c_void_p), ct.c_uint, ct.c_int],
            'cuCtxDestroy_v2': [ct.c_void_p],
            'cuCtxPushCurrent_v2': [ct.c_void_p],
            'cuCtxPopCurrent_v2': [ct.POINTER(ct.c_void_p)],
            'cuModuleLoadData': [ct.POINTER(ct.c_void_p), ct.c_void_p],
            'cuModuleUnload': [ct.c_void_p],
            'cuModuleGetFunction': [ct.POINTER(ct.c_void_p), ct.c_void_p, ct.c_char_p],
            'cuMemHostAlloc': [ct.POINTER(ct.c_void_p), ct.c_size_t, ct.c_uint],
            'cuMemFreeHost': [ct.c_void_p],
            'cuMemHostGetDevicePointer_v2': [ct.POINTER(pointer), ct.c_void_p, ct.c_uint],
            'cuLaunchKernel': [ct.c_void_p, ct.c_uint, ct.c_uint, ct.c_uint,
                               ct.c_uint, ct.c_uint, ct.c_uint, ct.c_uint,
                               ct.c_void_p, ct.POINTER(ct.c_void_p), ct.POINTER(ct.c_void_p)],
            'cuCtxSynchronize': [],
        }
        for name, args in signatures.items():
            fn = getattr(self._cuda, name)
            fn.argtypes, fn.restype = args, ct.c_int
        self._context = ct.c_void_p()
        self._module = ct.c_void_p()
        self._function = ct.c_void_p()
        self._allocations: list[ct.c_void_p] = []
        self._capacity = 0
        self._call('cuInit', 0)
        device = ct.c_int()
        self._call('cuDeviceGet', ct.byref(device), 0)
        # Block, rather than burn CPU while waiting; enable mapped pinned host buffers.
        self._call('cuCtxCreate_v2', ct.byref(self._context), 4 | 8, device)
        initialized = False
        try:
            ptx = ct.create_string_buffer(_PTX)
            self._call('cuModuleLoadData', ct.byref(self._module), ptx)
            self._call('cuModuleGetFunction', ct.byref(self._function), self._module, b'preprocess')
            self._sdiv, self._sdiv_device = self._allocate(256 * 4)
            table = np.zeros(256, dtype=np.uint32)
            table[1:] = np.rint((255 << 12) / np.arange(1, 256, dtype=np.float64))
            self._sdiv.view(np.uint32)[:] = table
            initialized = True
        finally:
            popped = ct.c_void_p()
            self._call('cuCtxPopCurrent_v2', ct.byref(popped))
            if not initialized:
                self.close()

    def _call(self, name: str, *args: Any) -> None:
        code = getattr(self._cuda, name)(*args)
        if code:
            error = ct.c_char_p()
            self._cuda.cuGetErrorName(code, ct.byref(error))
            raise RuntimeError(f'{name} failed: {code} {error.value!r}')

    def _allocate(self, size: int) -> tuple[np.ndarray, ct.c_uint64]:
        host, device = ct.c_void_p(), ct.c_uint64()
        self._call('cuMemHostAlloc', ct.byref(host), size, 2)
        self._allocations.append(host)
        self._call('cuMemHostGetDevicePointer_v2', ct.byref(device), host, 0)
        if host.value is None:
            raise RuntimeError('CUDA returned a null mapped host buffer')
        array = np.ctypeslib.as_array((ct.c_uint8 * size).from_address(host.value))
        return array, device

    def __call__(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError('CUDA preprocessing requires uint8 HxWx3 BGR')
        height, width = frame.shape[:2]
        count = height * width
        if not 0 < count <= 0xffffffff:
            raise ValueError('CUDA preprocessing requires 1..2^32-1 pixels')
        if not self._context.value:
            raise RuntimeError('CUDA preprocessor is closed')
        self._call('cuCtxPushCurrent_v2', self._context)
        try:
            if count > self._capacity:
                # Table is the first allocation; image buffers are reused at stable resolution.
                while len(self._allocations) > 1:
                    self._call('cuMemFreeHost', self._allocations[-1])
                    self._allocations.pop()
                self._capacity = 0
                self._input, self._input_device = self._allocate(count * 3)
                self._output, self._output_device = self._allocate(count * 2)
                self._capacity = count
            np.copyto(self._input[:count * 3].reshape(height, width, 3), frame)
            n = ct.c_uint(count)
            params = (ct.c_void_p * 4)(
                ct.cast(ct.byref(self._input_device), ct.c_void_p),
                ct.cast(ct.byref(self._output_device), ct.c_void_p),
                ct.cast(ct.byref(self._sdiv_device), ct.c_void_p),
                ct.cast(ct.byref(n), ct.c_void_p),
            )
            self._call('cuLaunchKernel', self._function, (count + 255) // 256, 1, 1,
                       256, 1, 1, 0, None, params, None)
            self._call('cuCtxSynchronize')
            planes = self._output[:count * 2].reshape(2, height, width)
            return planes[0].copy(), planes[1].copy()
        finally:
            popped = ct.c_void_p()
            self._call('cuCtxPopCurrent_v2', ct.byref(popped))

    def close(self) -> None:
        if not self._context.value:
            return
        self._call('cuCtxPushCurrent_v2', self._context)
        try:
            while self._allocations:
                self._call('cuMemFreeHost', self._allocations[-1])
                self._allocations.pop()
            if self._module.value:
                self._call('cuModuleUnload', self._module)
                self._module = ct.c_void_p()
        finally:
            popped = ct.c_void_p()
            self._call('cuCtxPopCurrent_v2', ct.byref(popped))
            self._call('cuCtxDestroy_v2', self._context)
            self._context = ct.c_void_p()

    def __enter__(self) -> CudaPreprocessor:
        return self

    def __exit__(
        self, exc_type: type[BaseException] | None, exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()
