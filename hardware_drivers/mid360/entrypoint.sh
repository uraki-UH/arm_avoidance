#!/bin/bash
# 通常起動・単発診断の共通ROS環境とシグナル転送
set -e
source /opt/livox_ws/install/setup.bash
exec "$@"
