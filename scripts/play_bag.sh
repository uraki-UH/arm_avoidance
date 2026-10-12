#!/usr/bin/env bash
set -euo pipefail

# メタデータと許可リストの照合による、必要トピック限定のROS 2 bag再生
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
topics_file="$script_dir/bag_topics.txt"
bag_path=''
is_dry_run=0
has_play_arguments=0
extra_topics=()
play_arguments=()
usage() {
    printf '%s\n' \
        '使い方: bash scripts/play_bag.sh BAG [--topics-file FILE] [--topic TOPIC] [--dry-run] [--loop] [再生オプション]' \
        'BAG: metadata.yamlを含むROS 2 bagディレクトリ（分割bagはディレクトリ全体）' \
        '--topics-file: 既定のbag_topics.txtに代わる許可リスト。1行1トピック、#はコメント、*と?の使用可能' \
        '--topic: 許可リストへの追加トピック／パターン。複数指定可能' \
        '--dry-run: 再生対象と実行コマンドの表示のみ。ROS起動なし' \
        '--loop、--rate 0.5、--clock、--remapなどを直接指定可能。BAGは再生オプションより前。従来の -- 区切りも利用可能'
}
fail() { printf '%s\n' "$*" >&2; exit 2; }
while (( $# > 0 )); do
    case "$1" in
        --topics-file|--topic)
            (( $# >= 2 )) && [[ -n "$2" && "$2" != --* ]] || fail "$1 に値が必要です"
            if [[ "$1" == --topics-file ]]; then topics_file="$2"; else extra_topics+=("$2"); fi
            shift 2 ;;
        --dry-run) is_dry_run=1; shift ;;
        --) shift; play_arguments+=("$@"); break ;;
        -h|--help) usage; exit 0 ;;
        -*) [[ -n "$bag_path" ]] || fail '再生オプションより前にbagを指定してください'
            play_arguments+=("$1"); has_play_arguments=1; shift ;;
        *) if [[ -z "$bag_path" ]]; then bag_path="$1"
            elif (( has_play_arguments )); then play_arguments+=("$1")
            else fail 'bagは1つだけ指定してください'; fi
            shift ;;
    esac
done
[[ -n "$bag_path" ]] || { usage >&2; exit 2; }
[[ -r "$bag_path/metadata.yaml" ]] || fail "metadata.yamlを読み込めません: $bag_path"
[[ -r "$topics_file" ]] || fail "トピック設定を読み込めません: $topics_file"
for argument in "${play_arguments[@]}"; do
    [[ "$argument" != --topics && "$argument" != --topics=* ]] || fail 'トピック指定は --topics-file または --topic を使用してください'
done

# YAMLの安全な読込み。照合失敗時の全トピック再生へのフォールバック禁止
selected_topics=$(python3 - "$bag_path/metadata.yaml" "$topics_file" "${extra_topics[@]}" <<'PY'
import fnmatch
import sys
try:
    import yaml
except ImportError:
    print('PyYAMLが必要です。python3-yamlを利用できる環境で実行してください。', file=sys.stderr)
    sys.exit(2)
try:
    with open(sys.argv[1], encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)['rosbag2_bagfile_information']
    available = {entry['topic_metadata']['name']: int(entry['message_count'])
                 for entry in metadata['topics_with_message_count']}
    with open(sys.argv[2], encoding='utf-8-sig') as stream:
        patterns = [line.split('#', 1)[0].strip() for line in stream]
    patterns = [item for item in patterns if item] + sys.argv[3:]
    selected = []
    for pattern in patterns:
        if not pattern.startswith('/') or any(char.isspace() for char in pattern):
            raise ValueError(f'トピック指定が不正: {pattern!r}')
        matches = [name for name, count in available.items()
                   if count > 0 and fnmatch.fnmatchcase(name, pattern)]
        if not matches:
            print(f'警告: 記録なし／0メッセージのため対象外: {pattern}', file=sys.stderr)
        for name in matches:
            if name not in selected:
                selected.append(name)
    if not selected:
        raise ValueError('再生対象が0件です。bagとトピック設定を確認してください（再生中止）')
    print('\n'.join(selected))
except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
    print(f'トピック選択失敗: {error}\nPython 3・PyYAMLとROS 2形式のmetadata.yamlが必要です。', file=sys.stderr)
    sys.exit(2)
PY
) || exit 2
mapfile -t topics <<< "$selected_topics"
command_args=(ros2 bag play "$bag_path" --read-ahead-queue-size 50 "${play_arguments[@]}" --topics "${topics[@]}")
printf '再生対象（%sトピック）:\n' "${#topics[@]}"
printf '  %s\n' "${topics[@]}"
printf '実行コマンド: '
printf '%q ' "${command_args[@]}"
printf '\n'
(( is_dry_run )) && exit 0

# 未設定のDocker環境のみ補完。ホストや別ディストリビューションの設定済み環境は保持
if ! command -v ros2 >/dev/null 2>&1; then
    set +u
    if [[ -r "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]]; then
        source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
    fi
    if [[ -r /ros2_ws/install/setup.bash ]]; then source /ros2_ws/install/setup.bash; fi
    set -u
fi
command -v ros2 >/dev/null 2>&1 || fail 'ROS 2環境をsourceするか、ROSコンテナ内で実行してください'
exec "${command_args[@]}"
