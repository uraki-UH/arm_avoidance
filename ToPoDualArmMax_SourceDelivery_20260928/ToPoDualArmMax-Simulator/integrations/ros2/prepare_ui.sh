#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.."

# 同時起動による依存配置・生成UIの競合防止
mkdir -p runtime
(
    flock -x 9
    if [[ ! -f app/generated/ros-results.js || ! -f app/vendor/three/build/three.module.js || ! -f app/vendor/three/build/three.core.js ]]; then
        printf '%s\n' '初回起動用のUI依存取得・ビルド中…'
        # Dockerビルド時のnpmキャッシュ優先。不足分のみ追加取得
        npm ci --prefer-offline --no-audit --no-fund
        npm run build
    fi
) 9>runtime/ui-build.lock
