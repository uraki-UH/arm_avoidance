#!/usr/bin/env bash

set -euo pipefail

repository_root="$(git rev-parse --show-toplevel)"
current_branch="$(git -C "${repository_root}" branch --show-current)"

if [[ -z "${current_branch}" ]]; then
    echo "error=detached_head"
    exit 2
fi

base_key="branch.${current_branch}.codex-base"
base_branch="$(git -C "${repository_root}" config --get "${base_key}" || true)"

if [[ -z "${base_branch}" ]]; then
    echo "worker=${current_branch} base=unset action=set_base"
    exit 2
fi

if ! git -C "${repository_root}" rev-parse --verify --quiet "${base_branch}^{commit}" >/dev/null; then
    echo "worker=${current_branch} base=${base_branch} error=base_not_found"
    exit 2
fi

read -r ahead_num behind_num < <(
    git -C "${repository_root}" rev-list --left-right --count "HEAD...${base_branch}"
)
dirty_num="$(git -C "${repository_root}" status --porcelain --untracked-files=normal | wc -l)"

root_path="$(git -C "${repository_root}" worktree list --porcelain | awk '/^worktree / {print substr($0, 10); exit}')"
root_branch="$(git -C "${root_path}" branch --show-current 2>/dev/null || true)"
is_root_mismatch="no"
if [[ -n "${root_branch}" && "${root_branch}" != "${base_branch}" ]]; then
    is_root_mismatch="yes"
fi

echo "worker=${current_branch} base=${base_branch} ahead=${ahead_num} behind=${behind_num} dirty=${dirty_num} root_branch=${root_branch:-unknown} is_root_mismatch=${is_root_mismatch}"
