#!/usr/bin/env bash
set -euo pipefail

# 別PC向けのCPU計算環境と、既存開発コンテナから独立したビルド領域
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
compose=(docker compose -f "$workspace_dir/docker/compose.simulator_cpu.yaml")
simulator_path=/ros2_ws/src/ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator
command_name="${1:---help}"
if (( $# > 0 )); then shift; fi
case "$command_name" in
    -h|--help|help)
        cat <<'USAGE'
使い方: bash scripts/dual_arm_cpu.sh <操作> [引数]
  build       CPUイメージ・基本ROSメッセージ・ブラウザUIの初回構築
  build-gng   GNG・VLUT・ROS表示ゲートウェイの追加構築（学習は別途）
  check       ROS・点群処理・CPU物理の有限時間動作確認
  check-gng   GNGパッケージ・学習済みデータの配置確認
  start       ROSブリッジとUIサーバーの前景起動（Ctrl+Cで停止）
  simulate    ブラウザなしの有限時間実行（引数はsimulate.pyへ転送）
  shell       ROS環境を読み込んだ一時コンテナのシェル
  stop        このCompose構成の停止（ビルド領域は保持）
例: bash scripts/dual_arm_cpu.sh simulate --avoidance none --duration 3
USAGE
        exit 0 ;;
    build|build-gng|check|check-gng|start|simulate|shell|stop) ;;
    *) printf '未対応の操作: %s\n' "$command_name" >&2; exit 2 ;;
esac
if [[ "$command_name" != simulate && $# != 0 ]]; then
    printf '%s\n' '追加引数はsimulateでのみ指定できます。' >&2
    exit 2
fi
command -v docker >/dev/null || { printf '%s\n' 'Docker EngineとCompose pluginが必要です。' >&2; exit 1; }
case "$command_name" in
    build)
        "${compose[@]}" build simulator
        exec "${compose[@]}" run --rm --no-deps simulator bash -ec '
            source /opt/ros/humble/setup.bash
            cd /ros2_ws
            colcon build --symlink-install --executor sequential \
                --paths src/MSG/ais_gng_feature_msgs src/ais_gng_cpu/src/ais_gng_msgs \
                        src/MSG/gng_control_msgs src/MSG/voxel_msgs \
                --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
            bash /ros2_ws/src/ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/prepare_ui.sh
        ' ;;
    build-gng)
        exec "${compose[@]}" run --rm --no-deps simulator bash -ec '
            source /opt/ros/humble/setup.bash
            cd /ros2_ws
            colcon build --symlink-install --executor sequential \
                --base-paths src/MSG src/ais_gng_cpu/src src/libs/arrow_visualization \
                    src/libs/voxel_idx src/pointcloud_sampling src/grasping_system \
                    src/gng_vlut_system src/ToPoFuzzy-Viewer/backend/src \
                --packages-up-to gng_vlut_system topo_fuzzy_viewer \
                --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
        ' ;;
    check)
        exec "${compose[@]}" run --rm --no-deps simulator bash -ec '
            source /opt/ros/humble/setup.bash
            source /ros2_ws/install/local_setup.bash
            python3 -c "import rclpy, tornado, scipy, mujoco, osqp; import ais_gng_msgs.msg, ais_gng_feature_msgs.msg, gng_control_msgs.srv, voxel_msgs.msg; print(\"ROS・CPU依存: OK\")"
            python3 integrations/ros2/native_points.py
            bash start_ros.sh --simulate --avoidance none --duration 1
        ' ;;
    check-gng)
        exec "${compose[@]}" run --rm --no-deps simulator bash -ec '
            source /opt/ros/humble/setup.bash
            source /ros2_ws/install/local_setup.bash
            ros2 pkg prefix gng_vlut_system
            ros2 pkg prefix topo_fuzzy_viewer
            PYTHONPATH=integrations/ros2 python3 -c "from gng_inputs import load_models, workspace; params, models = load_models(workspace / \"gng_vlut_system/config/topo_dual_arm_max_long.yaml\"); print(\"GNG学習済みデータ: OK\", list(models))"
        ' ;;
    start)
        exec "${compose[@]}" up --no-build simulator ;;
    simulate)
        exec "${compose[@]}" run --rm --no-deps simulator bash "$simulator_path/start_ros.sh" --simulate "$@" ;;
    shell)
        exec "${compose[@]}" run --rm --no-deps simulator bash -c '
            source /opt/ros/humble/setup.bash
            if [[ -f /ros2_ws/install/local_setup.bash ]]; then source /ros2_ws/install/local_setup.bash; fi
            exec bash
        ' ;;
    stop)
        exec "${compose[@]}" down ;;
esac
