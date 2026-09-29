#!/usr/bin/env bash
# Regenerates the checked-in golden screenshots under tests/goldens/ from
# the current code + scenarios (issue #14 stage 2). Run this when a visual
# change is intentional, then commit the updated PNGs alongside the change.
#
# Usage: scripts/update_golden_baselines.sh [build-dir]   (default: build)
set -euo pipefail

cd "$(dirname "$0")/.."
BUILD_DIR="${1:-build}"

cmake --build "$BUILD_DIR" --target tactics_visual_tests

LIBGL_ALWAYS_SOFTWARE=1 SDL_VIDEODRIVER=x11 \
  xvfb-run --auto-servernum --server-args='-screen 0 1280x1024x24' \
  "$BUILD_DIR/tactics_visual_tests" --update-baselines

echo "Done. Review the changed PNGs under tests/goldens/ and commit them."
