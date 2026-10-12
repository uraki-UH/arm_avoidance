#!/usr/bin/env bash
set -euo pipefail

# 従来の回避bag用入口。選択・再生処理はplay_bag.shへ集約
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
bag_path='/rosbag/fuzzy/arm_avoidance/rosbag2_2026_10_09-22_01_23-20261012T015534Z-1-002/rosbag2_2026_10_09-22_01_23'
enable_joint_states=0
enable_roi_voxels=1
enable_camera_tf=1
is_dry_run=0
has_play_arguments=0
play_arguments=()
selection_arguments=()
while (( $# > 0 )); do
    case "$1" in
        --joint-states) enable_joint_states=1; shift ;;
        --roi-voxels) enable_roi_voxels=1; shift ;;
        --dry-run) is_dry_run=1; selection_arguments+=(--dry-run); shift ;;
        --no-camera-tf) enable_camera_tf=0; shift ;;
        --topics-file|--topic)
            if (( $# < 2 )); then printf '%s に値が必要です\n' "$1" >&2; exit 2; fi
            selection_arguments+=("$1" "$2"); shift 2 ;;
        --) shift; play_arguments+=("$@"); break ;;
        -h|--help)
            printf '%s\n' '使い方: bash scripts/play_arm_avoidance_bag.sh [--roi-voxels] [--joint-states] [--no-camera-tf] [--dry-run] [bagディレクトリ] [--loop] [再生オプション]' 'カメラ取付TFも同時起動。配信済みの場合は --no-camera-tf。bagの変更は再生オプションより前。汎用入口: bash scripts/play_bag.sh BAG --help'
            exit 0 ;;
        -*) play_arguments+=("$1"); has_play_arguments=1; shift ;;
        *) if (( has_play_arguments )); then play_arguments+=("$1"); else bag_path="$1"; fi; shift ;;
    esac
done
if (( enable_roi_voxels )); then
    # 仮想ロボットでの回避用ROIと、記録ロボットの残像除去用自己マスク
    selection_arguments+=(--topic /topo_dual_arm_max_long/roi_voxels --topic /topo_dual_arm_max_long/self_voxel)
fi
if (( enable_joint_states )); then
    selection_arguments+=(--topic /topo_dual_arm_max_long/joint_states)
fi
bag_command=(bash "$script_dir/play_bag.sh" "$bag_path" "${selection_arguments[@]}" -- "${play_arguments[@]}")
if (( ! enable_camera_tf )); then exec "${bag_command[@]}"; fi
# 起動前の選択検証。bag不正・dry-run時の補完TF配信防止
if (( is_dry_run )); then
    "${bag_command[@]}"
    printf '%s\n' '同時起動: ros2 launch gng_vlut_system realsense_mount_tf.launch.py parent_frame:=topo_dual_arm_max_long/camera_link child_frame:=camera_link'
    exit 0
fi
bash "$script_dir/play_bag.sh" "$bag_path" "${selection_arguments[@]}" --dry-run -- "${play_arguments[@]}" >/dev/null
if ! command -v ros2 >/dev/null 2>&1; then
    set +u
    if [[ -r "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]]; then source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"; fi
    if [[ -r /ros2_ws/install/setup.bash ]]; then source /ros2_ws/install/setup.bash; fi
    set -u
fi
command -v ros2 >/dev/null 2>&1 || { printf '%s\n' 'ROS 2環境をsourceしてください' >&2; exit 2; }

# 自分で起動した2グループだけの監視・終了。標準入力とbagのキー操作の保持
exec python3 -c '
import os, signal, subprocess, sys, time

children = []
def stop_on_signal(signum, unused_frame):
    raise SystemExit(128 + signum)
signal.signal(signal.SIGINT, stop_on_signal)
signal.signal(signal.SIGTERM, stop_on_signal)
try:
    mount_command = ["ros2", "launch", "gng_vlut_system", "realsense_mount_tf.launch.py",
                     "parent_frame:=topo_dual_arm_max_long/camera_link", "child_frame:=camera_link"]
    print("同時起動: " + " ".join(mount_command), flush=True)
    children.append(subprocess.Popen(mount_command, start_new_session=True))
    children.append(subprocess.Popen(sys.argv[1:], start_new_session=True))
    while all(child.poll() is None for child in children):
        time.sleep(0.1)
    if children[1].poll() is not None:
        raise SystemExit(children[1].returncode)
    print("取付TFが終了したため、bag再生も停止します", file=sys.stderr)
    raise SystemExit(children[0].returncode or 1)
finally:
    # 後片付け中の追加Ctrl+Cによる中断防止
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for child in children:
            try:
                if signum == signal.SIGINT:
                    # launch側の子ノード停止処理を優先し、二重SIGINTを回避
                    if child.poll() is None: child.send_signal(signum)
                else:
                    os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass
        for child in children:
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pass
' "${bag_command[@]}"
