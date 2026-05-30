#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
  INSTALL_PREFIX="${HOME:-/root}/.local"
else
  INSTALL_PREFIX="${HOME}/.local"
fi

if ! command -v curl >/dev/null 2>&1; then
  echo "curl is required but was not found." >&2
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is required but was not found." >&2
  exit 1
fi

if ! command -v uv >/dev/null 2>&1; then
  echo "[install_urdf_spherizer] Installing uv into ${INSTALL_PREFIX}..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi

export PATH="${INSTALL_PREFIX}/bin:${PATH}"

if ! uv python list 2>/dev/null | grep -q '3.13'; then
  echo "[install_urdf_spherizer] Installing Python 3.13 via uv..."
  uv python install 3.13
fi

echo "[install_urdf_spherizer] Installing upstream urdf-spherizer..."
uv tool install --python 3.13 urdf-spherizer

echo "[install_urdf_spherizer] Done."
echo "Try:"
echo "  urdf-spherizer --help"
