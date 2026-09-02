#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "使用法: $0 取得先ディレクトリ" >&2
  exit 1
fi

eval_root="$1"
mkdir -p "$eval_root"

clone_repository() {
  local repo_url="$1"
  local directory_name="$2"
  local target_dir="$eval_root/$directory_name"
  if [[ -d "$target_dir/.git" ]]; then
    echo "取得済み: $target_dir"
    return
  fi
  git clone "$repo_url" "$target_dir"
}

if [[ ! -d "$eval_root/TEASER-plusplus/.git" ]]; then
  git clone --recursive https://github.com/MIT-SPARK/TEASER-plusplus.git \
    "$eval_root/TEASER-plusplus"
else
  echo "取得済み: $eval_root/TEASER-plusplus"
fi

clone_repository https://github.com/HKUST-Aerial-Robotics/G3Reg.git G3Reg
clone_repository https://github.com/ZhiChen902/SC2-PCR.git SC2-PCR
clone_repository https://github.com/Laka-3DV/TurboReg.git TurboReg
clone_repository https://github.com/NVlabs/FoundationPose.git FoundationPose
clone_repository https://github.com/bhsphd/FilterReg.git FilterReg
clone_repository https://github.com/SuperShrimp/FlashReg.git FlashReg

echo "取得完了: $eval_root"
