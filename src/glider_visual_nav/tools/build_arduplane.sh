#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
DEPS="$ROOT/build_visual_nav/ardupilot_deps"
mkdir -p "$DEPS"
AP_COMMIT=3fc7011a7d3dc047cbb17d8bd98ee94577d144c6
GZ_COMMIT=f116b03552d05c8d0b0bed5ed67033a1183121d1
if [[ ! -d "$DEPS/ardupilot/.git" ]]; then
  git clone --depth 1 --branch Plane-4.6.3 https://github.com/ArduPilot/ardupilot.git "$DEPS/ardupilot"
fi
test "$(git -C "$DEPS/ardupilot" rev-parse HEAD)" = "$AP_COMMIT"
git -C "$DEPS/ardupilot" submodule update --init --recursive --depth 1
if [[ ! -d "$DEPS/ardupilot_gazebo/.git" ]]; then
  git init "$DEPS/ardupilot_gazebo"
  git -C "$DEPS/ardupilot_gazebo" fetch --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git "$GZ_COMMIT"
  git -C "$DEPS/ardupilot_gazebo" checkout --detach FETCH_HEAD
fi
test "$(git -C "$DEPS/ardupilot_gazebo" rev-parse HEAD)" = "$GZ_COMMIT"
for PATCH in "$ROOT"/src/glider_visual_nav/simulation/patches/*.patch; do
if ! git -C "$DEPS/ardupilot" apply --reverse --check "$PATCH" 2>/dev/null; then
  git -C "$DEPS/ardupilot" apply --check "$PATCH"
  git -C "$DEPS/ardupilot" apply "$PATCH"
fi
done
python3 -m venv --system-site-packages "$DEPS/python"
"$DEPS/python/bin/pip" install pexpect future empy==3.3.4 pymavlink
(
  source "$DEPS/python/bin/activate"
  cd "$DEPS/ardupilot"
  python ./waf configure --board sitl
  python ./waf plane -j "${BUILD_JOBS:-3}"
)
source /opt/ros/jazzy/setup.bash
GZ_VERSION=harmonic cmake -S "$DEPS/ardupilot_gazebo" -B "$DEPS/gz_build"
cmake --build "$DEPS/gz_build" -j "${BUILD_JOBS:-3}"
