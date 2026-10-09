FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]
# 実機LiDAR専用依存関係
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git libapr1-dev libpcl-dev python3-yaml \
    python3-colcon-common-extensions ros-humble-ament-cmake-auto \
    ros-humble-pcl-conversions ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*
ARG sdk_rev=c0796f04c143143899c87a773d9f6b7136453c0b
ARG driver_rev=21445540f0d100dc86a7e6df312dd70bbdb4afdf
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /opt/livox_sdk \
    && cd /opt/livox_sdk && git checkout "$sdk_rev" \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j2 && cmake --install build && ldconfig
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git /opt/livox_ws/src/livox_ros_driver2 \
    && cd /opt/livox_ws/src/livox_ros_driver2 && git checkout "$driver_rev" \
    && cp package_ROS2.xml package.xml \
    && source /opt/ros/humble/setup.bash && cd /opt/livox_ws \
    && MAKEFLAGS=-j2 colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
COPY mid360.launch.py /opt/mid360/mid360.launch.py
CMD ["bash", "-c", "source /opt/livox_ws/install/setup.bash && exec ros2 launch /opt/mid360/mid360.launch.py"]
