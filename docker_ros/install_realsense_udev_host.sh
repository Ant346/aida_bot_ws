#!/usr/bin/env bash
# Installs Intel RealSense libusb udev rules on the Linux host.
# udev runs on the host; Docker bind-mounts /dev, so devices must be permitted here.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
RULES_SRC="${ROOT}/udev/99-realsense-libusb.rules"
RULES_DST=/etc/udev/rules.d/99-realsense-libusb.rules

if [[ ! -f "$RULES_SRC" ]]; then
  echo "Missing $RULES_SRC" >&2
  exit 1
fi

sudo install -m 0644 "$RULES_SRC" "$RULES_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Installed $RULES_DST"
echo "(Re)plug the RealSense USB cable if the device was already connected."
echo "Host tools outside Docker: add user to plugdev if you use MODE 0660 elsewhere: sudo usermod -aG plugdev \"\$USER\""
