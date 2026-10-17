#!/usr/bin/env bash
set -euo pipefail

# Install system dependencies for Trellis + MuJoCo Ninja builds on Ubuntu/Debian.
#
# Usage:
#   ./build+ubuntu.sh deps        # install packages only
#   ./build+ubuntu.sh build       # install packages + build vendor/mujoco
#   ./build+ubuntu.sh             # same as "build"

MODE="${1:-build}"

if [[ "$MODE" != "deps" && "$MODE" != "build" ]]; then
  echo "Usage: $0 [deps|build]"
  exit 2
fi

SUDO=""
if [[ "$(id -u)" -ne 0 ]]; then
  SUDO="sudo"
fi

echo "[deps] Installing Ubuntu/Debian packages..."
$SUDO apt-get update
$SUDO apt-get install -y \
  build-essential \
  ninja-build \
  pkg-config \
  python3 \
  libglfw3-dev \
  libgles2-mesa-dev \
  libegl1-mesa-dev \
  libx11-dev \
  libxrandr-dev \
  libxinerama-dev \
  libxcursor-dev \
  libxi-dev \
  glslang-tools \
  xvfb

if [[ "$MODE" == "deps" ]]; then
  echo "[deps] Done."
  exit 0
fi

echo "[build] Building vendored MuJoCo with Ninja..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
ninja -f build.ninja mujoco
echo "[build] Done."
