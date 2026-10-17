#!/usr/bin/env bash
set -euo pipefail

# Install system dependencies for Trellis + MuJoCo Ninja builds on
# RHEL/CentOS/Rocky/AlmaLinux/Fedora.
#
# Usage:
#   ./build+rhel.sh deps        # install packages only
#   ./build+rhel.sh build       # install packages + build vendor/mujoco
#   ./build+rhel.sh             # same as "build"

MODE="${1:-build}"

if [[ "$MODE" != "deps" && "$MODE" != "build" ]]; then
  echo "Usage: $0 [deps|build]"
  exit 2
fi

SUDO=""
if [[ "$(id -u)" -ne 0 ]]; then
  SUDO="sudo"
fi

PKG_MGR=""
if command -v dnf >/dev/null 2>&1; then
  PKG_MGR="dnf"
elif command -v yum >/dev/null 2>&1; then
  PKG_MGR="yum"
else
  echo "Error: neither dnf nor yum is available."
  exit 1
fi

echo "[deps] Installing RHEL/Fedora packages with $PKG_MGR..."
$SUDO "$PKG_MGR" install -y \
  gcc \
  gcc-c++ \
  make \
  ninja-build \
  pkgconf-pkg-config \
  python3 \
  glfw-devel \
  mesa-libGLES-devel \
  mesa-libEGL-devel \
  libX11-devel \
  libXrandr-devel \
  libXinerama-devel \
  libXcursor-devel \
  libXi-devel \
  glslang \
  xorg-x11-server-Xvfb

if [[ "$MODE" == "deps" ]]; then
  echo "[deps] Done."
  exit 0
fi

echo "[build] Building vendored MuJoCo with Ninja..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
ninja -f build.ninja mujoco
echo "[build] Done."
