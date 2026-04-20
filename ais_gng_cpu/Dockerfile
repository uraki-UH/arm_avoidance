FROM osrf/ros:humble-desktop-full

# 必要なパッケージのインストール
RUN apt update && apt install -y curl libykpiv-dev build-essential gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf gcc-aarch64-linux-gnu g++-aarch64-linux-gnu git

ENV DEBIAN_FRONTEND=noninteractive
# 必要であれば、タイムゾーン自体も設定しておく（日本時間の場合）
ENV TZ=Asia/Tokyo
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# colconとrosdepのインストール
RUN apt install -y python3-colcon-common-extensions python3-rosdep ros-dev-tools python3-pip ros-humble-pcl-ros

# opensslのビルドとインストール
RUN git clone https://github.com/openssl/openssl.git -b openssl-3.6.1
WORKDIR /openssl    
RUN ./Configure -static -fPIC --cross-compile-prefix="/usr/bin/x86_64-linux-gnu-" --prefix="/usr/local/openssl-x86_64" linux-x86_64 && \
    make clean && make && make install
RUN ./Configure -static -fPIC --cross-compile-prefix="/usr/bin/aarch64-linux-gnu-" --prefix="/usr/local/openssl-aarch64" linux-aarch64 && \
    make clean && make && make install

# PyTorchのインストール (CPU only)
RUN pip3 install torch==2.8.0 torchvision --index-url https://download.pytorch.org/whl/cpu
# PyGのインストール (CPU only)
RUN pip3 install pyg_lib torch_scatter torch_sparse torch_cluster torch_spline_conv -f https://data.pyg.org/whl/torch-2.8.0+cpu.html

# armhfアーキテクチャの追加
WORKDIR /etc/apt
RUN dpkg --add-architecture arm64
RUN echo "deb [arch=arm64] http://ports.ubuntu.com/ $(lsb_release -sc) main restricted" >> sources.list.d/arm64.list && \
    echo "deb [arch=arm64] http://ports.ubuntu.com/ $(lsb_release -sc)-updates main restricted" >> sources.list.d/arm64.list && \
    echo "deb [arch=arm64] http://ports.ubuntu.com/ $(lsb_release -sc) universe" >> sources.list.d/arm64.list && \
    echo "deb [arch=arm64] http://ports.ubuntu.com/ $(lsb_release -sc)-updates universe" >> sources.list.d/arm64.list && \
    echo "deb [arch=arm64] http://ports.ubuntu.com/ $(lsb_release -sc) multiverse" >> sources.list.d/arm64.list && \
    echo "deb [arch=arm64] http://ports.ubuntu.com/ $(lsb_release -sc)-updates multiverse" >> sources.list.d/arm64.list && \
    echo "deb [arch=arm64] http://ports.ubuntu.com/ $(lsb_release -sc)-backports main restricted universe multiverse" >> sources.list.d/arm64.list && \
    ARCH=$(dpkg --print-architecture) && \
    sed -i "s/deb /deb [arch=$ARCH] /g" sources.list
RUN apt update && apt install -y libykpiv-dev:arm64 openssl zip vim ros-humble-rosbag2-storage-mcap

# コマンドの追加
RUN echo '. /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'export ROS2_WS=ros2_ws' >> /root/.bashrc && \
    echo 'function cw() { cd /${ROS2_WS}; }' >> /root/.bashrc && \
    echo 'function cs() { cd /${ROS2_WS}/src; }' >> /root/.bashrc && \
    echo 'function cb() { cd /${ROS2_WS}; if [ -z $1 ]; then colcon build --symlink-install; else colcon build --symlink-install --packages-select $1; fi; if [ -f install/setup.bash ]; then . install/setup.bash; fi; cd -;}' >> /root/.bashrc && \
    echo 'function cbd() { cd /${ROS2_WS}; if [ -z $1 ]; then colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug; else colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select $1; fi; if [ -f install/setup.bash ]; then . install/setup.bash; fi; cd -;}' >> /root/.bashrc && \
    echo 'function ws() { if [ $1 ]; then ROS2_WS=$1_ws&&echo "switch ${ROS2_WS}"&&if [ -f /${ROS2_WS}/install/setup.bash ]; then . /${ROS2_WS}/install/setup.bash; fi; fi;}' >> /root/.bashrc && \
    echo "alias cl='cw && rm -rf ./build ./install ./log && cd -'" >> /root/.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then . /ros2_ws/install/setup.bash; fi' >> /root/.bashrc && \
    mkdir -p /ros2_ws/src

# ワークスペースの作成とパッケージのコピー
WORKDIR /ros2_ws

# エントリポイント
CMD ["/bin/bash"]
