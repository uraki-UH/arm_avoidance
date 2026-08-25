#!/usr/bin/env bash

set -euo pipefail

repository_root="$(git rev-parse --show-toplevel)"
current_branch="$(git -C "${repository_root}" branch --show-current)"
enable_rebase=false
set_base=""

usage() {
    echo "usage: $0 [--rebase] [--set-base <branch>]"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --rebase)
            enable_rebase=true
            shift
            ;;
        --set-base)
            if [[ $# -lt 2 ]]; then
                usage
                exit 2
            fi
            set_base="$2"
            shift 2
            ;;
        *)
            usage
            exit 2
            ;;
    esac
done

if [[ -z "${current_branch}" ]]; then
    echo "detached HEADでは基準ブランチを更新できません。" >&2
    exit 2
fi

if [[ -n "${set_base}" ]]; then
    if ! git -C "${repository_root}" rev-parse --verify --quiet "${set_base}^{commit}" >/dev/null; then
        echo "基準ブランチが見つかりません: ${set_base}" >&2
        exit 2
    fi
    git -C "${repository_root}" config "branch.${current_branch}.codex-base" "${set_base}"
    echo "worker=${current_branch} base=${set_base} updated=config_only"
    exit 0
fi

base_branch="$(git -C "${repository_root}" config --get "branch.${current_branch}.codex-base" || true)"
if [[ -z "${base_branch}" ]]; then
    echo "基準ブランチ未設定: $0 --set-base <branch>" >&2
    exit 2
fi

dirty_num="$(git -C "${repository_root}" status --porcelain --untracked-files=normal | wc -l)"
if [[ "${dirty_num}" -ne 0 ]]; then
    echo "dirtyなworktreeの自動同期は禁止: dirty=${dirty_num}" >&2
    exit 3
fi

read -r ahead_num behind_num < <(
    git -C "${repository_root}" rev-list --left-right --count "HEAD...${base_branch}"
)

if [[ "${behind_num}" -eq 0 ]]; then
    echo "worker=${current_branch} base=${base_branch} ahead=${ahead_num} behind=0 updated=noop"
    exit 0
fi

if [[ "${ahead_num}" -eq 0 ]]; then
    git -C "${repository_root}" merge --ff-only "${base_branch}"
    echo "worker=${current_branch} base=${base_branch} updated=fast_forward"
    exit 0
fi

if [[ "${enable_rebase}" != true ]]; then
    echo "基準ブランチと分岐済み: --rebaseの明示指定が必要です。ahead=${ahead_num} behind=${behind_num}" >&2
    exit 4
fi

git -C "${repository_root}" rebase "${base_branch}"
echo "worker=${current_branch} base=${base_branch} updated=rebase"
