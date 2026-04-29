FROM osrf/ros:humble-desktop-full

# 必要なパッケージのインストール
RUN apt update && apt install -y curl gnupg libykpiv-dev build-essential gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf gcc-aarch64-linux-gnu g++-aarch64-linux-gnu git nlohmann-json3-dev

ENV DEBIAN_FRONTEND=noninteractive
# 必要であれば、タイムゾーン自体も設定しておく（日本時間の場合）
ENV TZ=Asia/Tokyo
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# colconとrosdepのインストール
RUN apt install -y python3-colcon-common-extensions python3-rosdep ros-dev-tools python3-pip zip vim \
    ros-humble-rosbag2-storage-mcap ros-humble-pcl-ros \
    ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-xacro

# Node.js / npm for the frontend dev server
RUN mkdir -p /etc/apt/keyrings \
 && curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key \
    | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg \
 && echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_20.x nodistro main" \
    > /etc/apt/sources.list.d/nodesource.list \
 && apt update \
 && apt install -y nodejs \
 && rm -rf /var/lib/apt/lists/*

# PyTorchのインストール (CPU only)
RUN pip3 install torch==2.8.0 torchvision --index-url https://download.pytorch.org/whl/cpu
# PyGのインストール (CPU only)
RUN pip3 install pyg_lib torch_scatter torch_sparse torch_cluster torch_spline_conv -f https://data.pyg.org/whl/torch-2.8.0+cpu.html

# コマンドの追加
RUN echo '. /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'export ROS2_WS=ros2_ws' >> /root/.bashrc && \
    echo 'function cw() { cd /${ROS2_WS}; }' >> /root/.bashrc && \
    echo 'function cs() { cd /${ROS2_WS}/src; }' >> /root/.bashrc && \
    echo 'function cb() { cd /${ROS2_WS}; if [ -z $1 ]; then colcon build --symlink-install; else colcon build --symlink-install --packages-select $1; fi; . install/setup.bash && cd -;}' >> /root/.bashrc && \
    echo 'function cbd() { cd /${ROS2_WS}; if [ -z $1 ]; then colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug; else colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select $1; fi; . install/setup.bash && cd -;}' >> /root/.bashrc && \
    echo 'function ws() { if [ $1 ]; then ROS2_WS=$1_ws&&echo "switch ${ROS2_WS}"&&. /${ROS2_WS}/install/setup.bash;fi;}' >> /root/.bashrc && \
    echo "alias cl='cw && rm -rf ./build ./install ./log && cd -'" >> /root/.bashrc && \
    echo '. /ros2_ws/install/setup.bash' >> /root/.bashrc && \
    mkdir -p /ros2_ws/src

# ワークスペースの作成とパッケージのコピー
WORKDIR /ros2_ws

WORKDIR /ros2_ws/src/ToPoFuzzy-Viewer/frontend
COPY ToPoFuzzy-Viewer/frontend/package*.json /ros2_ws/src/ToPoFuzzy-Viewer/frontend/
RUN npm ci

# エントリポイント
CMD ["/bin/bash"]
