FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    gnupg \
    libpcl-dev \
    libssl-dev \
    nlohmann-json3-dev \
    pkg-config \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    zlib1g-dev \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /etc/apt/keyrings \
 && curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key \
    | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg \
 && echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_20.x nodistro main" \
    > /etc/apt/sources.list.d/nodesource.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends nodejs \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/topofuzzy

COPY ToPoFuzzy-Viewer/backend/ /opt/topofuzzy/backend/
COPY ToPoFuzzy-Viewer/config/ /opt/topofuzzy/config/
COPY ToPoFuzzy-Viewer/common/ /opt/topofuzzy/common/

# Prefer backend/src/ais_gng_msgs from ToPoFuzzy-Viewer and ignore the duplicate in external ais_gng.
RUN if [ -d /opt/topofuzzy/backend/src/ais_gng/ais_gng_msgs ]; then \
      touch /opt/topofuzzy/backend/src/ais_gng/ais_gng_msgs/COLCON_IGNORE; \
    fi

WORKDIR /opt/topofuzzy/backend
RUN source /opt/ros/humble/setup.bash \
 && colcon build --packages-up-to topo_fuzzy_viewer --symlink-install

WORKDIR /opt/topofuzzy/frontend
COPY ToPoFuzzy-Viewer/frontend/package*.json /opt/topofuzzy/frontend/
RUN npm ci
COPY ToPoFuzzy-Viewer/frontend/ /opt/topofuzzy/frontend/

WORKDIR /opt/topofuzzy
CMD ["/bin/bash"]
