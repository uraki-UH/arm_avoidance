#!/usr/bin/env bash
set -euo pipefail

# メタデータと許可リストの照合による、必要トピック限定のROS 2 bag再生
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
topics_file=''
config_file=''
bag_path=''
is_dry_run=0
has_play_arguments=0
extra_topics=()
play_arguments=()
usage() {
    printf '%s\n' \
        '使い方: bash scripts/play_bag.sh BAG [--topics-file FILE] [--topic TOPIC] [--dry-run] [--loop] [再生オプション]' \
        '設定から起動: bash scripts/play_bag.sh --config FILE [BAG] [--dry-run] [再生オプション]' \
        'BAG: metadata.yamlを含むROS 2 bagディレクトリ（分割bagはディレクトリ全体）' \
        '--topics-file: 既定のbag_topics.txtに代わる許可リスト。1行1トピック、#はコメント、*と?の使用可能' \
        '--topic: 許可リストへの追加トピック／パターン。複数指定可能' \
        '--dry-run: 再生対象と実行コマンドの表示のみ。ROS起動なし' \
        '--loop、--rate 0.5、--clock、--remapなどを直接指定可能。BAGは再生オプションより前。従来の -- 区切りも利用可能'
}
fail() { printf '%s\n' "$*" >&2; exit 2; }
while (( $# > 0 )); do
    case "$1" in
        --config)
            (( $# >= 2 )) && [[ -n "$2" && "$2" != --* ]] || fail "$1 に値が必要です"
            config_file="$2"; shift 2 ;;
        --topics-file|--topic)
            (( $# >= 2 )) && [[ -n "$2" && "$2" != --* ]] || fail "$1 に値が必要です"
            if [[ "$1" == --topics-file ]]; then topics_file="$2"; else extra_topics+=("$2"); fi
            shift 2 ;;
        --dry-run) is_dry_run=1; shift ;;
        --) shift; play_arguments+=("$@"); break ;;
        -h|--help) usage; exit 0 ;;
        -*) [[ -n "$bag_path" || -n "$config_file" ]] || fail '再生オプションより前にbagまたは--configを指定してください'
            play_arguments+=("$1"); has_play_arguments=1; shift ;;
        *) if (( has_play_arguments )); then play_arguments+=("$1")
            elif [[ -z "$bag_path" ]]; then bag_path="$1"
            else fail 'bagは1つだけ指定してください'; fi
            shift ;;
    esac
done
[[ -n "$bag_path" || -n "$config_file" ]] || { usage >&2; exit 2; }
for argument in "${play_arguments[@]}"; do
    [[ "$argument" != --topics && "$argument" != --topics=* ]] || fail 'トピック指定は --topics-file または --topic を使用してください'
done

# YAMLの安全な読込み。照合失敗時の全トピック再生へのフォールバック禁止
player_arguments=$(python3 - "$config_file" "$bag_path" "$topics_file" "$script_dir/bag_topics.txt" "${#extra_topics[@]}" "${extra_topics[@]}" "${play_arguments[@]}" <<'PY'
import fnmatch
from pathlib import Path
import re
import sys
try:
    import yaml
except ImportError:
    print('PyYAMLが必要です。python3-yamlを利用できる環境で実行してください。', file=sys.stderr)
    sys.exit(2)
try:
    config_file, bag_path, topics_file, default_topics_file = sys.argv[1:5]
    num_extra_topics = int(sys.argv[5])
    extra_topics = sys.argv[6:6+num_extra_topics]
    play_arguments = sys.argv[6+num_extra_topics:]
    config = {}
    config_arguments = []
    if config_file:
        with open(config_file, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        if not isinstance(config, dict) or set(config)-{'bag_path', 'topics', 'enable_loop', 'remap'}:
            raise ValueError('再生YAMLのキーはbag_path・topics・enable_loop・remapです')
        configured_path = config.get('bag_path', '')
        if not isinstance(configured_path, str):
            raise ValueError('bag_pathには文字列が必要です')
        if not bag_path and configured_path:
            path = Path(configured_path).expanduser()
            bag_path = str(path if path.is_absolute() else Path(config_file).resolve().parent/path)
        enable_loop = config.get('enable_loop', False)
        if not isinstance(enable_loop, bool):
            raise ValueError('enable_loopには真偽値が必要です')
        if enable_loop and '--loop' not in play_arguments:
            config_arguments.append('--loop')
        remap = config.get('remap', {})
        if not isinstance(remap, dict):
            raise ValueError('remapには変更前と変更後のトピック名の対応が必要です')
        for source, target in remap.items():
            if any(not isinstance(name, str) or not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*', name)
                   for name in (source, target)):
                raise ValueError('remapには絶対トピック名が必要です')
        # CLIでのリマップ指定時は、YAML側の対応表を一括置換
        if remap and '--remap' not in play_arguments and not any(item.startswith('--remap=') for item in play_arguments):
            config_arguments += ['--remap', *[source+':='+target for source, target in remap.items()]]
    if not bag_path:
        raise ValueError('bag_pathまたはBAGの指定が必要です')
    with open(Path(bag_path)/'metadata.yaml', encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)['rosbag2_bagfile_information']
    available = {entry['topic_metadata']['name']: int(entry['message_count'])
                 for entry in metadata['topics_with_message_count']}
    if config_file and not topics_file:
        patterns = config.get('topics')
        if not isinstance(patterns, list) or not patterns or any(not isinstance(item, str) for item in patterns):
            raise ValueError('topicsには空でないトピック配列が必要です')
    else:
        with open(topics_file or default_topics_file, encoding='utf-8-sig') as stream:
            patterns = [line.split('#', 1)[0].strip() for line in stream]
        patterns = [item for item in patterns if item]
    patterns = patterns+extra_topics
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
    arguments = [bag_path, '--read-ahead-queue-size', '50', *config_arguments, *play_arguments, '--topics', *selected]
    if any('\n' in item or '\r' in item or '\0' in item for item in arguments):
        raise ValueError('引数中の改行・NULは使用できません')
    print('\n'.join(arguments))
except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
    print(f'トピック選択失敗: {error}\nPython 3・PyYAMLとROS 2形式のmetadata.yamlが必要です。', file=sys.stderr)
    sys.exit(2)
PY
) || exit 2
mapfile -t player_args <<< "$player_arguments"
command_args=(ros2 bag play "${player_args[@]}")
for idx in "${!player_args[@]}"; do
    if [[ "${player_args[$idx]}" == --topics ]]; then topics=("${player_args[@]:$((idx+1))}"); break; fi
done
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
