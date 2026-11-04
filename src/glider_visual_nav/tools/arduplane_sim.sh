#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
PKG="$ROOT/src/glider_visual_nav"
source "$PKG/tools/demo_env.bash"
export ROS_DOMAIN_ID=67 GZ_PARTITION=glider_arduplane
DEPS="$ROOT/build_visual_nav/ardupilot_deps"
PLANE="${ARDUPLANE_BIN:-$DEPS/ardupilot/build/sitl/bin/arduplane}"
PLUGIN="${ARDUPILOT_GZ_PLUGIN_DIR:-$DEPS/gz_build}"
export GZ_SIM_RESOURCE_PATH="$PKG/simulation/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGIN:$ROOT/install_visual_nav/glider_visual_nav/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
case "${1:-run}" in
  dashboard) exec python3 "$PKG/tools/flight_dashboard.py" ;;
  gui) exec gz sim -g ;;
  fly) exec gz service -s /evaluation/launch --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 5000 --req "x:16 y:0 z:0" ;;
  passive) exec gz sim -s -r --headless-rendering "$PKG/simulation/worlds/glider_visual.sdf" ;;
  run) ;;
  *) echo "Usage: $0 [run|passive|dashboard|gui|fly]" >&2; exit 2 ;;
esac
if [[ -f "$DEPS/python/bin/activate" ]]; then source "$DEPS/python/bin/activate"; fi
test -x "$PLANE" || { echo "Missing ArduPlane binary: $PLANE" >&2; exit 1; }
test -f "$PLUGIN/libArduPilotPlugin.so" || { echo "Missing ArduPilotPlugin: $PLUGIN" >&2; exit 1; }
RUN="$ROOT/log_visual_nav/arduplane_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$RUN"
pids=()
cleanup() {
  if (( ${#pids[@]} )); then
    kill -TERM -- "${pids[@]/#/-}" 2>/dev/null || true
    for _ in {1..10}; do
      local alive=false
      for pid in "${pids[@]}"; do
        if kill -0 -- "-$pid" 2>/dev/null; then alive=true; fi
      done
      if ! $alive; then break; fi
      sleep .2
    done
    kill -KILL -- "${pids[@]/#/-}" 2>/dev/null || true
    wait || true
  fi
}
trap cleanup EXIT
trap "exit 130" INT
trap "exit 143" TERM
start() {
  local log=$1; shift
  setsid "$@" > "$RUN/$log.log" 2>&1 &
  pids+=("$!")
}
start gazebo gz sim -s -r --headless-rendering "$PKG/simulation/worlds/arduplane_glider_visual.sdf"
cd "$RUN"
gcs=none
if [[ "${ARDUPLANE_ENABLE_CONTROL:-false}" == true ]]; then gcs=udpclient:127.0.0.1:14551; fi
start plane "$PLANE" --model JSON --defaults "${ARDUPLANE_PARAMS:-$PKG/config/arduplane_glider.parm}" --serial0 udpclient:127.0.0.1:14550 --serial1 "$gcs" --serial2 none --home 47.397742,8.545594,0,90
start bridge ros2 run ros_gz_bridge parameter_bridge   "/world/glider_visual_flight/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"   "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"   "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"   "/world/glider_visual_flight/model/glider_visual/link/base_link/sensor/magnetometer_sensor/magnetometer@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer" "/evaluation/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry"   --ros-args -r /world/glider_visual_flight/clock:=/clock -r /world/glider_visual_flight/model/glider_visual/link/base_link/sensor/magnetometer_sensor/magnetometer:=/glider/magnetic_field
start mavlink python3 "$PKG/tools/ardupilot_mavlink_bridge_node" --ros-args   -p use_sim_time:=true -p initial_body_height_m:=145.25 -p flight_regime:=unpowered_glide   -p send_external_velocity:=true -p send_external_pose:=true -p set_local_origin:=true -p publish_magnetometer:=false   -p enable_attitude_control:="${ARDUPLANE_ENABLE_CONTROL:-false}"
start navigation ros2 launch glider_visual_nav navigation.launch.py   "params_file:=$PKG/config/arduplane_navigation.yaml" orientation_mode:=imu_aided
echo "ArduPlane logs: $RUN"
wait -n "${pids[@]}"
