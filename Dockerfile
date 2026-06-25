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

# ROS2 / 開発 / rosbag / RealSense 関連
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-dev-tools \
    python3-pip \
    zip \
    vim \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-pcl-ros \
    ros-humble-rosbridge-server \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-camera-msgs \
    && rm -rf /var/lib/apt/lists/*

# Node.js / npm for the frontend dev server
RUN mkdir -p /etc/apt/keyrings \
 && curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key \
    | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg \
 && echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_20.x nodistro main" \
    > /etc/apt/sources.list.d/nodesource.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends nodejs \
 && rm -rf /var/lib/apt/lists/*

# Emscripten (WASM build for gng_wasm_core)
RUN cd /opt && git clone https://github.com/emscripten-core/emsdk.git \
 && cd emsdk && ./emsdk install latest && ./emsdk activate latest
ENV PATH="/opt/emsdk:/opt/emsdk/upstream/emscripten:${PATH}"

# PyTorch CPU only
RUN pip3 install torch==2.8.0 torchvision --index-url https://download.pytorch.org/whl/cpu

# PyG CPU only
RUN pip3 install pyg_lib torch_scatter torch_sparse torch_cluster torch_spline_conv \
    -f https://data.pyg.org/whl/torch-2.8.0+cpu.html

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
WORKDIR /ros2_ws

# frontend dependencies
WORKDIR /ros2_ws/src/ToPoFuzzy-Viewer/frontend
COPY ToPoFuzzy-Viewer/frontend/package*.json /ros2_ws/src/ToPoFuzzy-Viewer/frontend/
RUN npm ci

WORKDIR /ros2_ws/src

CMD ["/bin/bash"]