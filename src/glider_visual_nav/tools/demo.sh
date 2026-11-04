#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$ROOT/src/glider_visual_nav/tools/demo_env.bash"
LOG="$ROOT/log_visual_nav/demo"
mkdir -p "$LOG"
case "${1:-help}" in
  ardupilot) shift; exec bash "$ROOT/src/glider_visual_nav/tools/arduplane_sim.sh" "$@" ;;
  start)
    if [[ -f "$LOG/launch.pid" ]] && kill -0 "$(cat "$LOG/launch.pid")" 2>/dev/null; then
      echo "Demo already running. Use status, gui, camera or focus."
      bash "$ROOT/src/glider_visual_nav/tools/demo.sh" dashboard
      exit 0
    fi
    if pgrep -f '^gz sim -s .*visual_nav_flight.sdf' >/dev/null; then
      echo "An existing flight server is running. Stop its launch before starting another demo." >&2
      exit 1
    fi
    WORK="$ROOT/build_visual_nav/demo_$(date +%Y%m%d_%H%M%S)_$$"
    nohup setsid ros2 launch glider_visual_nav flight.launch.py flight_regime:=cruise_const_alt "px4_workdir:=$WORK" > "$LOG/launch.log" 2>&1 < /dev/null &
    echo $! > "$LOG/launch.pid"
    echo "Started. Log: $LOG/launch.log"
    bash "$ROOT/src/glider_visual_nav/tools/demo.sh" dashboard
    ;;
  gui)
    nohup gz sim -g > "$LOG/gui.log" 2>&1 < /dev/null &
    echo "Gazebo opened. Run focus after the scene loads."
    ;;
  camera)
    nohup ros2 run rqt_image_view rqt_image_view /camera/image_raw > "$LOG/camera.log" 2>&1 < /dev/null &
    ;;
  dashboard)
    if pgrep -f "^python3 $ROOT/src/glider_visual_nav/tools/flight_dashboard.py$" >/dev/null; then
      echo "Dashboard already running."
      exit 0
    fi
    nohup setsid python3 "$ROOT/src/glider_visual_nav/tools/flight_dashboard.py" > "$LOG/dashboard.log" 2>&1 < /dev/null &
    echo "Flight dashboard opened (map, camera, course and evaluation metrics)."
    ;;
  focus)
    gz service -s /gui/move_to --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 5000 --req 'data: "rc_cessna_visual"'
    ;;
  follow)
    gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 5000 --req 'data: "rc_cessna_visual"'
    ;;
  status)
    timeout 12 ros2 topic echo /glider/interface_state std_msgs/msg/String --once --no-daemon
    timeout 12 ros2 topic echo /camera/image_raw sensor_msgs/msg/Image --once --field width --no-daemon
    ;;
  fly)
    echo "Only run after status reports ACTIVE. One-time simulator air-launch fixture."
    gz service -s /evaluation/launch --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 5000 --req 'x: 16.0 y: 0.0 z: 0.0'
    ;;
  stop)
    if [[ -f "$LOG/launch.pid" ]]; then
      PID=$(cat "$LOG/launch.pid")
      if [[ "$PID" =~ ^[0-9]+$ ]] && ps -p "$PID" -o args= | grep -q 'ros2 launch glider_visual_nav flight.launch.py'; then
        kill -INT -- "-$PID"
        echo "Stopping demo process group $PID."
      else
        echo "Recorded demo is no longer running."
      fi
    fi
    ;;
  *) echo "Usage: bash $0 {start|status|gui|camera|dashboard|ardupilot|focus|follow|fly|stop}" ;;
esac
