FROM ros:humble-ros-base

# 国内镜像。ROS2 源用 deb822 格式且把续期过的公钥内联在文件里，所以只原地改 URI，
# 顺便去掉 deb-src：镜像站的 Sources.gz 常与 Release 声明的大小对不上，留着它会让
# apt-get update 整体失败，而我们只装二进制包。
#
# 用阿里云而不是 USTC：2026-07 起 mirrors.ustc.edu.cn 在这台机器上全线不可达——
# 直连被本地 DNS 劫持到 10.0.0.1，走 SOCKS 代理也超时。阿里云的 ubuntu-ports、
# ros2 和 pypi 三个源都实测可用（需经代理）。
# pypi.org 与 files.pythonhosted.org 同样不可达，所以 pip 也必须指到阿里云，
# 写进 /etc/pip.conf 而不是命令行参数：容器里后续手动 pip 也才有源可用。
RUN set -eux; \
    printf '%s\n' \
      'deb https://mirrors.aliyun.com/ubuntu-ports/ jammy main restricted universe multiverse' \
      'deb https://mirrors.aliyun.com/ubuntu-ports/ jammy-updates main restricted universe multiverse' \
      'deb https://mirrors.aliyun.com/ubuntu-ports/ jammy-backports main restricted universe multiverse' \
      'deb https://mirrors.aliyun.com/ubuntu-ports/ jammy-security main restricted universe multiverse' \
      > /etc/apt/sources.list; \
    sed -i \
      -e 's|^Types: .*|Types: deb|' \
      -e 's|^URIs: .*|URIs: https://mirrors.aliyun.com/ros2/ubuntu|' \
      /etc/apt/sources.list.d/ros2.sources; \
    printf '%s\n' 'APT::Install-Recommends "false";' 'APT::Install-Suggests "false";' \
      > /etc/apt/apt.conf.d/99no-recommends; \
    printf '%s\n' '[global]' 'index-url = https://mirrors.aliyun.com/pypi/simple/' \
      'trusted-host = mirrors.aliyun.com' > /etc/pip.conf

# 一层装完，失败时能从日志里一眼看出缺哪个包。pip 与 apt 同层：pip 走 socks5h 代理
# 需要 PySocks（由 python3-socks 提供），分层写会让"先装依赖再用"这件事依赖层序，
# 合在一起才是自洽的。
RUN set -eux; \
    apt-get update; \
    apt-get install -y \
      build-essential cmake git gdb \
      less vim nano sudo \
      iproute2 iputils-ping net-tools usbutils can-utils \
      adb \
      ffmpeg \
      libusb-1.0-0-dev libyaml-cpp-dev \
      python3-colcon-common-extensions python3-pip python3-socks \
      python3-flask python3-opencv python3-numpy python3-scipy \
      python3-yaml python3-pytest \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-ros2controlcli \
      ros-humble-realtime-tools \
      ros-humble-pluginlib \
      ros-humble-robot-state-publisher \
      ros-humble-xacro \
      ros-humble-tf2-ros \
      ros-humble-tf2-geometry-msgs \
      ros-humble-cv-bridge \
      ros-humble-can-msgs \
      ros-humble-control-msgs \
      ros-humble-trajectory-msgs \
      ros-humble-diagnostic-msgs \
      ros-humble-visualization-msgs \
      ros-humble-interactive-markers \
      ros-humble-ros2launch \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-yaml-cpp-vendor \
      ros-humble-pinocchio \
      ros-humble-librealsense2 \
      ros-humble-realsense2-camera \
      ros-humble-realsense2-camera-msgs \
      ros-humble-realsense2-description \
      ros-humble-ament-cmake-gtest \
      ros-humble-ament-cmake-pytest \
      ros-humble-ament-cmake-python \
      ros-humble-ament-lint-auto \
      ros-humble-ament-lint-common \
      ros-humble-ament-index-cpp \
      ros-humble-ament-index-python \
      ros-humble-rosidl-default-generators \
      ros-humble-rosidl-generator-dds-idl; \
    pip3 install --no-cache-dir \
      'python-can>=4.0' 'canalystii>=0.1' 'libusb-package>=1.0.30' pyserial \
      'onnxruntime>=1.16' 'numpy>=1.21.6,<1.25' \
      'aiohttp>=3.9' 'cryptography'; \
    rm -rf /var/lib/apt/lists/*
# canalystii 与 libusb-package 是 CAN-SDK 的 [canalystii] extra，只供 Python 调试
# 链路使用；生产链路是 C++ 的 canalystii_native_bridge，不依赖它们。
# realsense2-* 是 G1 头顶 RealSense D435i 的官方驱动（src/head_sensors 用它）。相机走 USB
# 直连本机 NX，不是 DDS 设备；容器已经是 privileged + /dev:/dev，不需要额外装 udev 规则。
# 头部 Livox MID-360 反过来不需要任何本地驱动：机器人内部的 lidar_driver 服务直接发
# /utlidar/* 的标准 sensor_msgs，所以这里没有 livox SDK。
# adb + aiohttp + cryptography 供 src/g1_lower_body_policy/vr 的 WebXR 桥接用。WebXR 只在
# 安全上下文里可用，所以头显接入有两条并存的路：
#   - HTTPS 直连 https://<本机IP>:8443，证书由 `ros2 run g1_lower_body_policy make_vr_cert`
#     签（用 cryptography），不经过 adb，适合长时间使用；
#   - `adb reverse tcp:8000` 后头显访问 http://localhost:8000。容器是 network_mode: host，
#     规则建在容器里就等价于建在宿主上。
# onnxruntime 供 g1_lower_body_policy 的下肢策略层推理用。CPU 版就够：57->512->256->128->15 的 MLP 在 50 Hz 上不到 0.1 ms。
# numpy 的上下界都是被逼出来的，不能放开：
#   上界 <1.25 —— apt 的 python3-opencv、python3-scipy 和 ros-humble-cv-bridge 都是
#     against numpy 1.21 编译的 .so。numpy 2.0 换了 C ABI，升上去这三个一律
#     "_ARRAY_API not found"；1.25~1.26 虽然能跑，但 scipy 1.8 会每次 import 都告警。
#     onnxruntime 不带这个约束就会顺手把 numpy 拉到 2.x。
#   下界 >=1.21.6 —— onnxruntime 自己的要求，系统自带的 1.21.5 差一点。

# 与宿主同 UID/GID：工作区是 bind mount，不对齐会把源码文件写成 root 所有
ARG USER_NAME=unitree
ARG USER_UID=1000
ARG USER_GID=1000
RUN set -eux; \
    if getent group "${USER_GID}" >/dev/null; then \
      existing_group="$(getent group "${USER_GID}" | cut -d: -f1)"; \
      groupmod -n "${USER_NAME}" "${existing_group}"; \
    else \
      groupadd -g "${USER_GID}" "${USER_NAME}"; \
    fi; \
    if getent passwd "${USER_UID}" >/dev/null; then \
      existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
      usermod -l "${USER_NAME}" -d "/home/${USER_NAME}" -m -g "${USER_GID}" "${existing_user}"; \
    else \
      useradd -m -u "${USER_UID}" -g "${USER_GID}" -s /bin/bash "${USER_NAME}"; \
    fi; \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER_NAME}"; \
    chmod 0440 "/etc/sudoers.d/${USER_NAME}"

# 环境定义在工作区的 scripts/env.sh 里，不 COPY 进镜像：工作区总是挂在 /workspace，
# 直接用那一份就只有一个定义，改了立刻生效、不用重建镜像。代价是这个镜像离开工作区
# 挂载就跑不起来——本来也没意义。
RUN printf '%s\n' \
      '# VS Code "Reopen in Container" 开的终端不走 ENTRYPOINT，只读 ~/.bashrc。' \
      '[ -f /workspace/scripts/env.sh ] && source /workspace/scripts/env.sh' \
      >> "/home/${USER_NAME}/.bashrc"

USER ${USER_NAME}
ENV ROS_DISTRO=humble \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["/workspace/scripts/env.sh"]
CMD ["bash"]
