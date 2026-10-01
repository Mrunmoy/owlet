#!/usr/bin/env bash
# Fetch component datasheets listed in sources.env into this directory.
# Idempotent: skips files that already exist locally.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ ! -f sources.env ]]; then
    echo "sources.env not found in $SCRIPT_DIR" >&2
    exit 1
fi

while IFS='=' read -r local_name url; do
    # Skip blank lines and comments
    [[ -z "$local_name" || "$local_name" =~ ^[[:space:]]*# ]] && continue
    # Trim whitespace
    local_name="${local_name## }"; local_name="${local_name%% }"
    url="${url## }"; url="${url%% }"
    [[ -z "$url" ]] && continue

    if [[ -f "$local_name" ]]; then
        echo "[skip] $local_name (exists)"
        continue
    fi
    echo "[get ] $local_name <- $url"
    curl --fail --location --silent --show-error --output "$local_name" "$url"
done < sources.env

echo "done."
