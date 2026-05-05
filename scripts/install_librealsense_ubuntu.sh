#!/usr/bin/env bash
# Intel RealSense SDK 2.0 — APT install for Ubuntu LTS (official repo).
# Docs: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
set -euo pipefail

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  exec sudo --preserve-env=DEBIAN_FRONTEND "$0" "$@"
fi

apt-get update
apt-get install -y apt-transport-https ca-certificates curl gnupg

mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc |
  gpg --dearmor -o /etc/apt/keyrings/librealsenseai.gpg

CODENAME="$(lsb_release -cs)"
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo ${CODENAME} main" \
  > /etc/apt/sources.list.d/librealsense.list

apt-get update
apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev

echo "Done. Connect the camera and run: realsense-viewer"
echo "Kernel check: modinfo uvcvideo | grep version"
