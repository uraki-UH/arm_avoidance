#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
bash integrations/ros2/prepare_ui.sh
exec python3 integrations/ros2/start_simulator.py "$@"
