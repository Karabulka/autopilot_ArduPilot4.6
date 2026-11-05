#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
TOOLS="$ROOT/src/glider_visual_nav/tools"
source "$TOOLS/demo_env.bash"
export ROS_DOMAIN_ID=67 GZ_PARTITION=glider_arduplane
LOG="$ROOT/log_visual_nav/ardupilot_demo"
mkdir -p "$LOG"
case "${1:-help}" in
  start)
    if [[ -f "$LOG/pid" ]] && kill -0 "$(cat "$LOG/pid")" 2>/dev/null; then
      echo "ArduPlane demo is already running."; exit 0
    fi
    if pgrep -f "^$ROOT/build_visual_nav/ardupilot_deps/ardupilot/build/sitl/bin/arduplane --model" >/dev/null; then
      echo "Another ArduPlane trial is running; finish it first." >&2; exit 1
    fi
    nohup setsid bash "$TOOLS/arduplane_sim.sh" run > "$LOG/launch.log" 2>&1 < /dev/null &
    echo $! > "$LOG/pid"
    echo "ArduPlane started in monitor mode. Log: $LOG/launch.log"
    ;;
  gui|dashboard|fly) exec bash "$TOOLS/arduplane_sim.sh" "$1" ;;
  focus|follow)
    service=/gui/move_to
    if [[ "$1" == follow ]]; then service=/gui/follow; fi
    exec gz service -s "$service" --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 5000 --req 'data: "glider_visual"'
    ;;
  status)
    timeout 12 ros2 topic echo /glider/interface_state std_msgs/msg/String --once --no-daemon
    timeout 12 ros2 topic echo /glider/odometry nav_msgs/msg/Odometry --once --field pose.pose.position --no-daemon
    ;;
  flight)
    exec python3 "$TOOLS/glider_smoke.py" --arduplane --guided --duration "${2:-90}" --output "$ROOT/log_visual_nav/demo_guided_$(date +%Y%m%d_%H%M%S).json"
    ;;
  stop)
    if [[ -f "$LOG/pid" ]]; then
      pid=$(cat "$LOG/pid")
      if [[ "$pid" =~ ^[0-9]+$ ]] && ps -p "$pid" -o args= | grep -q "$TOOLS/arduplane_sim.sh run"; then
        kill -TERM -- "-$pid"
      fi
    fi
    ;;
  ardupilot) shift; exec bash "$TOOLS/arduplane_sim.sh" "$@" ;;
  px4) shift; exec bash "$TOOLS/demo_px4.sh" "$@" ;;
  *) echo "ArduPlane: $0 {start|gui|dashboard|focus|follow|status|stop}"
     echo "Automatic SITL trial with normal arming: $0 flight [60..90]"
     echo "Legacy PX4: $0 px4 start"
     echo "Do not run start and flight simultaneously."
     ;;
esac
