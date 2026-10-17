#!/usr/bin/env bash
set -euo pipefail

# 別PC向けのCPU計算環境と、既存開発コンテナから独立したビルド領域
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
compose=(docker compose -f "$workspace_dir/docker/compose.simulator_cpu.yaml")
command_name="${1:---help}"
if (( $# > 0 )); then shift; fi
case "$command_name" in
    -h|--help|help)
        cat <<'USAGE'
使い方: bash scripts/dual_arm_cpu.sh <操作> [引数]
  build       YAML構成に対応するCPUイメージ・ROSパッケージ・UIの構築
  check       YAML構成に対応する依存・短時間物理・学習データの確認
  start       ROSブリッジとUIサーバーの前景起動（Ctrl+Cで停止）
  simulate    ブラウザなしの有限時間実行（引数はsimulate.pyへ転送）
  shell       ROS環境を読み込んだ一時コンテナのシェル
  stop        このCompose構成の停止（ビルド領域は保持）
例: bash scripts/dual_arm_cpu.sh simulate --avoidance none --duration 3
設定: docker/compose.simulator_cpu.yaml の x-cpu-workflow
USAGE
        exit 0 ;;
    build-gng|check-gng)
        printf '%s\n' 'GNG専用操作は廃止しました。YAMLのenable_gngを設定し、build／checkを使用してください。' >&2
        exit 2 ;;
    build|check|start|simulate|shell|stop) ;;
    *) printf '未対応の操作: %s\n' "$command_name" >&2; exit 2 ;;
esac
if [[ "$command_name" != simulate && $# != 0 ]]; then
    printf '%s\n' '追加引数はsimulateでのみ指定できます。' >&2
    exit 2
fi
command -v docker >/dev/null || { printf '%s\n' 'Docker EngineとCompose pluginが必要です。' >&2; exit 1; }
run_workflow() {
    exec "${compose[@]}" run --rm --no-deps simulator bash -ec '
        source /opt/ros/humble/setup.bash
        if [[ -f /ros2_ws/install/local_setup.bash ]]; then source /ros2_ws/install/local_setup.bash; fi
        exec python3 /ros2_ws/src/scripts/dual_arm_cpu.py "$@"
    ' cpu-workflow "$@"
}
case "$command_name" in
    build)
        "${compose[@]}" build simulator
        run_workflow build ;;
    check)
        run_workflow check ;;
    start)
        exec "${compose[@]}" up --no-build simulator ;;
    simulate)
        run_workflow simulate "$@" ;;
    shell)
        exec "${compose[@]}" run --rm --no-deps simulator bash -c '
            source /opt/ros/humble/setup.bash
            if [[ -f /ros2_ws/install/local_setup.bash ]]; then source /ros2_ws/install/local_setup.bash; fi
            exec bash
        ' ;;
    stop)
        exec "${compose[@]}" down ;;
esac
