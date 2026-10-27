#!/usr/bin/env python3
"""Check CUDA Driver API access without launching a model or robot task."""

import ctypes
import sys


def main():
    cuda = ctypes.CDLL("libcuda.so.1")
    device_ptr_type = ctypes.c_uint64
    signatures = {
        "cuGetErrorName": [ctypes.c_int, ctypes.POINTER(ctypes.c_char_p)],
        "cuGetErrorString": [ctypes.c_int, ctypes.POINTER(ctypes.c_char_p)],
        "cuInit": [ctypes.c_uint],
        "cuDriverGetVersion": [ctypes.POINTER(ctypes.c_int)],
        "cuDeviceGetCount": [ctypes.POINTER(ctypes.c_int)],
        "cuDeviceGet": [ctypes.POINTER(ctypes.c_int), ctypes.c_int],
        "cuDeviceGetName": [ctypes.c_void_p, ctypes.c_int, ctypes.c_int],
        "cuCtxCreate_v2": [
            ctypes.POINTER(ctypes.c_void_p), ctypes.c_uint, ctypes.c_int,
        ],
        "cuCtxDestroy_v2": [ctypes.c_void_p],
        "cuMemAlloc_v2": [ctypes.POINTER(device_ptr_type), ctypes.c_size_t],
        "cuMemFree_v2": [device_ptr_type],
        "cuMemcpyHtoD_v2": [device_ptr_type, ctypes.c_void_p, ctypes.c_size_t],
        "cuMemcpyDtoH_v2": [ctypes.c_void_p, device_ptr_type, ctypes.c_size_t],
        "cuCtxSynchronize": [],
    }
    for name, arguments in signatures.items():
        function = getattr(cuda, name)
        function.restype = ctypes.c_int
        function.argtypes = arguments

    def check(result, operation):
        if result != 0:
            details = []
            for lookup in (cuda.cuGetErrorName, cuda.cuGetErrorString):
                text = ctypes.c_char_p()
                lookup_result = lookup(result, ctypes.byref(text))
                if lookup_result == 0 and text.value:
                    details.append(text.value.decode("utf-8", errors="replace"))
                else:
                    details.append(f"error lookup returned {lookup_result}")
            raise RuntimeError(f"{operation} failed: {result} ({'; '.join(details)})")
        print(f"PASS: {operation} returned CUDA_SUCCESS", flush=True)

    with open("/proc/self/maps", encoding="utf-8") as maps:
        paths = sorted({
            line.split()[-1] for line in maps if "/libcuda.so" in line
        })
    if not paths or any("/stubs/" in path for path in paths):
        raise RuntimeError(f"Cannot confirm a non-stub CUDA driver mapping: {paths}")
    print(f"Loaded CUDA driver: {', '.join(paths)}", flush=True)

    check(cuda.cuInit(0), "cuInit")
    version = ctypes.c_int()
    check(cuda.cuDriverGetVersion(ctypes.byref(version)), "cuDriverGetVersion")
    print(f"CUDA driver API version: {version.value}", flush=True)
    count = ctypes.c_int()
    check(cuda.cuDeviceGetCount(ctypes.byref(count)), "cuDeviceGetCount")
    if count.value < 1:
        raise RuntimeError("CUDA reported no GPU devices")
    print(f"CUDA GPU count: {count.value}", flush=True)
    devices = []
    for ordinal in range(count.value):
        device = ctypes.c_int()
        check(cuda.cuDeviceGet(ctypes.byref(device), ordinal), f"cuDeviceGet({ordinal})")
        name = ctypes.create_string_buffer(256)
        check(cuda.cuDeviceGetName(name, len(name), device), "cuDeviceGetName")
        print(f"GPU {ordinal}: {name.value.decode('utf-8')}", flush=True)
        devices.append(device.value)

    context = ctypes.c_void_p()
    allocation = device_ptr_type()
    size = 4096
    expected = bytes((index * 17 + 3) % 256 for index in range(size))
    source = (ctypes.c_ubyte * size).from_buffer_copy(expected)
    destination = (ctypes.c_ubyte * size)()
    try:
        check(cuda.cuCtxCreate_v2(ctypes.byref(context), 0, devices[0]), "cuCtxCreate_v2")
        check(cuda.cuMemAlloc_v2(ctypes.byref(allocation), size), "cuMemAlloc_v2 (4096 bytes)")
        check(cuda.cuMemcpyHtoD_v2(allocation, source, size), "cuMemcpyHtoD_v2")
        check(cuda.cuMemcpyDtoH_v2(destination, allocation, size), "cuMemcpyDtoH_v2")
        check(cuda.cuCtxSynchronize(), "cuCtxSynchronize")
        if bytes(destination) != expected:
            raise RuntimeError("4096-byte host/GPU round-trip data mismatch")
        print("PASS: 4096-byte host/GPU round-trip data matches", flush=True)
    finally:
        try:
            if allocation.value:
                check(cuda.cuMemFree_v2(allocation), "cuMemFree_v2")
        finally:
            if context.value:
                check(cuda.cuCtxDestroy_v2(context), "cuCtxDestroy_v2")
    print("PASS: CUDA Driver API checks completed; GPU inference was NOT tested.", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (OSError, AttributeError, RuntimeError) as error:
        print(f"FAIL: {error}", file=sys.stderr, flush=True)
        sys.exit(1)
