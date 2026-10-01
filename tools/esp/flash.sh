#!/usr/bin/env bash
# Thin wrapper: build + flash the owlet firmware.
# Usage: tools/esp/flash.sh [PORT]   (defaults to the auto-detected Espressif device)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "$REPO_ROOT/build.py" --esp --flash ${1:+"$1"}
