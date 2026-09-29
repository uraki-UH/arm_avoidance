#!/usr/bin/env bash
set -euo pipefail

# AMD GPUのコンテキスト喪失回避用のNVIDIA描画経路
if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi -L >/dev/null 2>&1; then
    printf '%s\n' '利用可能なNVIDIA GPUを確認できません。' >&2
    exit 1
fi

chrome_path="$(command -v google-chrome || command -v google-chrome-stable || true)"
if [[ -z "$chrome_path" ]]; then
    printf '%s\n' 'Google Chromeが見つかりません。' >&2
    exit 1
fi

egl_vendor_file='/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
if [[ ! -r "$egl_vendor_file" ]]; then
    printf '%s\n' 'NVIDIA EGLドライバの設定ファイルが見つかりません。' >&2
    exit 1
fi

# 旧OpenGL経路で起動済みのChromeへの転送を防ぐ専用プロファイル
profile_dir="${viewer_profile_dir:-${XDG_CACHE_HOME:-$HOME/.cache}/topofuzzy-viewer-nvidia-egl}"
viewer_url="${1:-http://localhost:5173}"
if (( $# > 0 )); then shift; fi
# チャットからコピーされたMarkdownリンクのURL抽出
markdown_url_pattern='^\[[^]]*\]\((https?://[^[:space:]]+)\)$'
if [[ "$viewer_url" =~ $markdown_url_pattern ]]; then
    viewer_url="${BASH_REMATCH[1]}"
fi
if [[ ! "$viewer_url" =~ ^https?://[^[:space:]]+$ ]]; then
    printf '%s\n' 'http:// または https:// で始まるURLを指定してください。' >&2
    exit 2
fi
printf '開くURL: %s\n' "$viewer_url"
exec env __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
    __EGL_VENDOR_LIBRARY_FILENAMES="$egl_vendor_file" \
    "$chrome_path" \
    --user-data-dir="$profile_dir" \
    --use-gl=angle --use-angle=gl-egl \
    --no-first-run --no-default-browser-check \
    "$@" \
    "$viewer_url"
