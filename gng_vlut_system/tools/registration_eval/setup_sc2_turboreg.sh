#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 3 ]]; then
  echo "使用法: $0 依存先ディレクトリ SC2-PCRのディレクトリ TurboRegのディレクトリ" >&2
  exit 1
fi

python_root="$1"
sc2_dir="$2"
turbo_dir="$3"

python3 -m pip install --target "$python_root" --upgrade torch \
  --index-url https://download.pytorch.org/whl/cu128
PYTHONPATH="$python_root" python3 -c \
  'import torch; print(f"cuda_available={torch.cuda.is_available()}"); print(f"gpu={torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"なし\"}")'
PYTHONPATH="$python_root" python3 -m pip install --target "$python_root" \
  --no-deps --no-build-isolation "$turbo_dir/bindings"
PYTHONPATH="$python_root:$sc2_dir" python3 -c \
  'from SC2_PCR import Matcher; print("SC2-PCRのimport成功")'
PYTHONPATH="$python_root" python3 -c \
  'import turboreg_gpu; print("TurboRegのimport成功")'
