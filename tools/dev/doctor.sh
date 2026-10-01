#!/usr/bin/env bash
# Verify the Nix dev shell has every tool owlet needs. Run after `nix develop`.

set -u

REQUIRED=(
    cmake
    ninja
    gcc
    g++
    git
    python3
    idf.py        # ESP-IDF entry point
    esptool.py    # ESP flashing
    clang-format
)

missing=0
for bin in "${REQUIRED[@]}"; do
    if command -v "$bin" >/dev/null 2>&1; then
        printf "  %-18s OK  (%s)\n" "$bin" "$(command -v "$bin")"
    else
        printf "  %-18s MISSING\n" "$bin"
        missing=$((missing + 1))
    fi
done

if (( missing > 0 )); then
    echo
    echo "$missing tool(s) missing. Are you inside 'nix develop'?" >&2
    exit 1
fi

echo
echo "All required tools found."
