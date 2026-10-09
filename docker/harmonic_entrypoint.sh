#!/bin/bash
set -e

# ROS本体とインストール済みHarmonic実行パッケージの読込み。
source /opt/ros/jazzy/setup.bash
source /opt/gng_harmonic/setup.bash
exec "$@"
