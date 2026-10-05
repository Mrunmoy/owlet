#!/usr/bin/env bash
# One-time setup on Linux: let your user flash and talk to owlet heads
# without sudo. Needs sudo itself, once, to install the rule.
#
#   sudo tools/dev/install-udev.sh
#
# Then unplug the board and plug it in again.

set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rule="99-owlet.rules"
target="/etc/udev/rules.d/$rule"

if [[ $EUID -ne 0 ]]; then
    echo "This installs $target and has to run as root:" >&2
    echo "    sudo $0" >&2
    exit 1
fi

install -m 0644 "$here/$rule" "$target"
udevadm control --reload-rules
udevadm trigger
echo "Installed $target."
echo "Unplug the board and plug it in again for the new permissions to apply."
