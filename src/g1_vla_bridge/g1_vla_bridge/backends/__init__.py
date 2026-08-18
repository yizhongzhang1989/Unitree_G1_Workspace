"""各家 VLA 服务的请求封装，一个模块一个 VLA

模块必须导出 ``SPEC`` / ``PARAMETERS`` / ``create(params)``，见
``g1_vla_bridge.vla_backend.VlaBackend``。选哪个走 ``vla_backend`` 这个 ROS 参数。
"""
