#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
deps="$task_root/build_visual_nav/deps"
mkdir -p "$deps"
agent_source="$deps/Micro-XRCE-DDS-Agent"
if [ ! -d "$agent_source/.git" ]; then
  git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$agent_source"
fi
test "$(git -C "$agent_source" rev-parse HEAD)" = 73622810d984349b80bbac0ef55fc0b694d62222
cmake -S "$agent_source" -B "$deps/agent_build" \
  -DUAGENT_SUPERBUILD=OFF -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_USE_SYSTEM_FASTCDR=ON \
  -DUAGENT_P2P_PROFILE=OFF -DUAGENT_CED_PROFILE=OFF -DUAGENT_LOGGER_PROFILE=OFF \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_root/install_visual_nav/xrce_agent"
cmake --build "$deps/agent_build" -j4
cmake --install "$deps/agent_build"
python3 "$task_root/src/glider_visual_nav/tools/prepare_px4_messages.py"
