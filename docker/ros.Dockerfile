FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-rosidl-default-generators \
    ros-humble-tf2-ros \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libegl1 \
    libgles2 \
    libglu1-mesa \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/xray

COPY docker/overlay.repos /opt/xray/overlay.repos
RUN mkdir -p /opt/xray/overlay_ws/src && \
    vcs import /opt/xray/overlay_ws/src < /opt/xray/overlay.repos && \
    source /opt/ros/humble/setup.bash && \
    colcon --log-base /opt/xray/overlay_ws/log build \
    --merge-install \
    --base-paths /opt/xray/overlay_ws/src \
    --build-base /opt/xray/overlay_ws/build \
    --install-base /opt/xray/overlay_ws/install

WORKDIR /opt/xray/ws
COPY ros2_ws/src /opt/xray/ws/src
RUN source /opt/ros/humble/setup.bash && \
    source /opt/xray/overlay_ws/install/setup.bash && \
    colcon --log-base /opt/xray/ws/log build \
    --merge-install \
    --base-paths /opt/xray/ws/src \
    --build-base /opt/xray/ws/build \
    --install-base /opt/xray/ws/install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
