FROM ros:humble-ros-base

# 国内镜像。ROS2 源用 deb822 格式且把续期过的公钥内联在文件里，所以只原地改 URI，
# 顺便去掉 deb-src：USTC 的 Sources.gz 与 Release 声明的大小对不上，留着它会让
# apt-get update 整体失败，而我们只装二进制包。
RUN set -eux; \
    printf '%s\n' \
      'deb https://mirrors.ustc.edu.cn/ubuntu-ports/ jammy main restricted universe multiverse' \
      'deb https://mirrors.ustc.edu.cn/ubuntu-ports/ jammy-updates main restricted universe multiverse' \
      'deb https://mirrors.ustc.edu.cn/ubuntu-ports/ jammy-backports main restricted universe multiverse' \
      'deb https://mirrors.ustc.edu.cn/ubuntu-ports/ jammy-security main restricted universe multiverse' \
      > /etc/apt/sources.list; \
    sed -i \
      -e 's|^Types: .*|Types: deb|' \
      -e 's|^URIs: .*|URIs: https://mirrors.ustc.edu.cn/ros2/ubuntu|' \
      /etc/apt/sources.list.d/ros2.sources; \
    printf '%s\n' 'APT::Install-Recommends "false";' 'APT::Install-Suggests "false";' \
      > /etc/apt/apt.conf.d/99no-recommends

# 一层装完，失败时能从日志里一眼看出缺哪个包
RUN set -eux; \
    apt-get update; \
    apt-get install -y \
      build-essential cmake git gdb \
      less vim nano sudo \
      iproute2 iputils-ping net-tools usbutils can-utils \
      libusb-1.0-0-dev libyaml-cpp-dev \
      python3-colcon-common-extensions python3-pip \
      python3-flask python3-opencv python3-numpy python3-scipy \
      python3-yaml python3-can python3-pytest \
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
      ros-humble-ament-cmake-gtest \
      ros-humble-ament-cmake-pytest \
      ros-humble-ament-cmake-python \
      ros-humble-ament-lint-auto \
      ros-humble-ament-lint-common \
      ros-humble-ament-index-cpp \
      ros-humble-ament-index-python \
      ros-humble-rosidl-default-generators \
      ros-humble-rosidl-generator-dds-idl; \
    rm -rf /var/lib/apt/lists/*

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

COPY entrypoint.sh /usr/local/bin/g1-entrypoint
RUN chmod +x /usr/local/bin/g1-entrypoint

USER ${USER_NAME}
ENV ROS_DISTRO=humble \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["/usr/local/bin/g1-entrypoint"]
CMD ["bash"]
