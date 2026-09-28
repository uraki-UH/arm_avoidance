#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
command -v node >/dev/null 2>&1 || { echo "Install Node.js 22 or later: https://nodejs.org/" >&2; exit 1; }
exec node app/server.mjs
