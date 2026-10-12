#!/usr/bin/env bash
set -eo pipefail
invocation_dir="$PWD"
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
workspace_source_dir="$(cd ../.. && pwd)"
# ROS workspace/src配置では、ソース側の古いinstallではなくworkspaceのinstall。
if [[ "${workspace_source_dir##*/}" == src ]]; then
    workspace_install_dir="$workspace_source_dir/../install"
else
    workspace_install_dir="$workspace_source_dir/install"
fi
if [[ -r "$workspace_install_dir/local_setup.bash" ]]; then
    # 独自メッセージ・ノードと共有ライブラリのワークスペース環境。
    source "$workspace_install_dir/local_setup.bash"
fi
export TOPO_ROS_SETUP_READY=1
if [[ "${1:-}" == --simulate ]]; then
    # ブラウザ非依存のMuJoCo・回避CLI。UI生成・管理サービス起動なし。
    shift
    cli_path="$PWD/integrations/ros2/simulate.py"
    cd -- "$invocation_dir"
    exec python3 "$cli_path" "$@"
fi
if [[ "${1:-}" == --bridge ]]; then
    # 単体ブリッジも同じROS環境入口。UIの再生成・管理サービス起動なし。
    shift
    exec python3 integrations/ros2/pointcloud_bridge.py "$@"
fi
bash integrations/ros2/prepare_ui.sh
exec python3 integrations/ros2/start_simulator.py "$@"
