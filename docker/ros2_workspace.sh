#!/usr/bin/env bash

# 開発コンテナの標準構築。独立プロジェクト・任意部品の自動探索なし。
_build_ros_workspace() {
    local build_type="$1"
    shift
    local workspace_path="/${ROS2_WS:-ros2_ws}"
    local -a source_paths package_options=()
    mapfile -t source_paths < <(python3 "$workspace_path/src/scripts/workspace_layout.py" ros-paths)
    if (( ${#source_paths[@]} == 0 )); then return 1; fi
    if (( $# > 0 )); then package_options=(--packages-up-to "$@"); fi
    (cd -- "$workspace_path" && colcon build --symlink-install --base-paths "${source_paths[@]}" \
        "${package_options[@]}" --cmake-args "-DCMAKE_BUILD_TYPE=$build_type") || return
    source "$workspace_path/install/setup.bash"
}

cb() { _build_ros_workspace Release "$@"; }
cbd() { _build_ros_workspace Debug "$@"; }
