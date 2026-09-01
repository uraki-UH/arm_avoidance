FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Tokyo

# タイムゾーン設定
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# 基本パッケージ
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    libykpiv-dev \
    build-essential \
    gcc-arm-linux-gnueabihf \
    g++-arm-linux-gnueabihf \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    git \
    nlohmann-json3-dev \
    usbutils \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# ROS2 / 開発 / URDF関連
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-dev-tools \
    python3-pip \
    zip \
    unzip \
    vim \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# bashrc 設定
RUN echo '. /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'export ROS2_WS=ros2_ws' >> /root/.bashrc && \
    echo "alias sh='source /opt/ros/humble/setup.bash'" >> /root/.bashrc && \
    echo "alias sw='source /ros2_ws/install/setup.bash'" >> /root/.bashrc && \
    echo 'function cw() { cd /${ROS2_WS}; }' >> /root/.bashrc && \
    echo 'function cs() { cd /${ROS2_WS}/src; }' >> /root/.bashrc && \
    echo 'function cb() { cd /${ROS2_WS}; if [ -z $1 ]; then colcon build --symlink-install; else colcon build --symlink-install --packages-select $1; fi; . install/setup.bash && cd -; }' >> /root/.bashrc && \
    echo 'function cbd() { cd /${ROS2_WS}; if [ -z $1 ]; then colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug; else colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select $1; fi; . install/setup.bash && cd -; }' >> /root/.bashrc && \
    echo 'function ws() { if [ $1 ]; then ROS2_WS=$1_ws && echo "switch ${ROS2_WS}" && . /${ROS2_WS}/install/setup.bash; fi; }' >> /root/.bashrc && \
    echo "alias cl='cw && rm -rf ./build ./install ./log && cd -'" >> /root/.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then . /ros2_ws/install/setup.bash; fi' >> /root/.bashrc && \
    mkdir -p /ros2_ws/src

# ワークスペース
WORKDIR /ros2_ws/src

CMD ["/bin/bash"]
