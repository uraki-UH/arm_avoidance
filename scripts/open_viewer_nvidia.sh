#!/usr/bin/env bash
set -euo pipefail

# 描画経路の選択。双腕用のauto指定時のみMesaへの切替
viewer_gpu="${viewer_gpu:-nvidia}"
case "$viewer_gpu" in
    auto|nvidia|mesa) ;;
    *) printf '%s\n' 'viewer_gpu は auto / nvidia / mesa を指定してください。' >&2; exit 2 ;;
esac
if [[ "$viewer_gpu" != mesa ]]; then
    if command -v nvidia-smi >/dev/null 2>&1 && nvidia_status="$(nvidia-smi -L 2>&1)"; then
        viewer_gpu=nvidia
    else
        printf '%s\n' "NVIDIAを利用できません: ${nvidia_status:-nvidia-smi がありません}" >&2
        if [[ "${nvidia_status:-}" == *'Driver/library version mismatch'* ]]; then
            printf '%s\n' 'NVIDIAのカーネルドライバとライブラリが不一致です。保存後のホスト再起動が必要です。' >&2
        fi
        if [[ "$viewer_gpu" == nvidia ]]; then exit 1; fi
        viewer_gpu=mesa
        printf '%s\n' '今回はMesa経由のGPU描画へ切り替えます。' >&2
    fi
fi

chrome_path="$(command -v google-chrome || command -v google-chrome-stable || true)"
if [[ -z "$chrome_path" ]]; then
    printf '%s\n' 'Google Chromeが見つかりません。' >&2
    exit 1
fi

if [[ "$viewer_gpu" == nvidia ]]; then
    egl_vendor_file='/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
    gpu_env=(__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia)
else
    egl_vendor_file='/usr/share/glvnd/egl_vendor.d/50_mesa.json'
    gpu_env=(__NV_PRIME_RENDER_OFFLOAD=0 __GLX_VENDOR_LIBRARY_NAME=mesa)
fi
if [[ ! -r "$egl_vendor_file" ]]; then
    printf '%s\n' 'EGLドライバの設定ファイルが見つかりません。' >&2
    exit 1
fi

# 旧OpenGL経路で起動済みのChromeへの転送を防ぐ専用プロファイル
profile_dir="${viewer_profile_dir:-${XDG_CACHE_HOME:-$HOME/.cache}/topofuzzy-viewer-${viewer_gpu}-egl}"
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
printf '描画経路: %s\n' "$viewer_gpu"
# ROSライブラリ探索パスをブラウザへ持ち込まない起動環境
exec env -u LD_LIBRARY_PATH "${gpu_env[@]}" \
    __EGL_VENDOR_LIBRARY_FILENAMES="$egl_vendor_file" \
    "$chrome_path" \
    --user-data-dir="$profile_dir" \
    --use-gl=angle --use-angle=gl-egl \
    --no-first-run --no-default-browser-check \
    "$@" \
    "$viewer_url"
