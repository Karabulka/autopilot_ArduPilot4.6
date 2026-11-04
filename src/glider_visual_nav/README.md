# glider_visual_nav

Standalone ROS 2 Jazzy / C++17 package. The OpenCV mathematical library has no ROS dependency.
No GPS, PX4 or simulator-state inputs. Configuration is validated at startup and read-only;
restart the nodes together to change orientation_mode or calibration parameters.

## Contracts

W/odom is ENU; B/base_link is FLU. Yaw zero east, positive left; positive roll banks right.
FLU Euler pitch is positive nose-down. Optical x is image-right, y image-down, z down toward ground.
R_BC = [[0,-1,0],[-1,0,0],[0,0,-1]], det=+1. The optical node owns the static optical TF
and camera_offset_body. Avoid duplicate TF publishers.

| Relative topic | Type | Contract |
| --- | --- | --- |
| camera/image_raw | sensor_msgs/Image | MONO8-compatible raw image, optical frame, acquisition stamp |
| camera/camera_info | sensor_msgs/CameraInfo | optical frame, matching dimensions, K overrides fallback |
| glider/imu | sensor_msgs/Imu | imu_aided only: base_link FLU gyro and FLU-to-ENU unit quaternion |
| glider/magnetic_field | sensor_msgs/MagneticField | physical calibrated base_link FLU vector in tesla |
| glider/visual_velocity | geometry_msgs/TwistWithCovarianceStamped | base_link m/s, interval midpoint stamp |
| glider/odometry | nav_msgs/Odometry | ENU pose, FLU twist; odom -> base_link TF |
| glider/target_pose | geometry_msgs/PoseStamped | next waypoint x/y, altitude z, RPY=(roll, minus pitch-up, LOS course) |

Inputs must share the ROS clock; Sprint 2 owns /clock and bridges. Timestamp zero is valid.
Magnetic/IMU samples are interpolated or taken from a bounded-age endpoint. Invalid/stale inputs
withhold output. Navigation resets its local origin on rewind; forward gaps hold position across
the unknown interval and inflate covariance. Guidance withholds commands on stale odometry,
insufficient speed, or completion of a non-looping route.
CameraInfo supports pinhole K and plumb_bob/rational_polynomial distortion, undistorted using K.
Unsupported ROI/binning/fisheye are rejected. Fallback calibration requires configured dimensions.
Calibration changes reset the image pair. Sensor inputs use SensorDataQoS; internal outputs use
reliable depth 10. All topics can be remapped.

## Modes and limitations

homography_mag uses forward/backward LK, RANSAC, calibrated in-plane rotation/translation and
finite yaw/lever-arm compensation at horizontal h=const. Inconsistent scale/projective changes
are rejected. This is not general 6-DoF homography decomposition; banked/pitched flight requires
imu_aided or a future full visual-pose estimator. Heading is magnetic with east-positive declination.

imu_aided uses per-feature plane depth, differential interaction matrices, B_i*omega_C subtraction
and omega_B x r_BC correction. IMU data is sampled at the image interval midpoint. This is a
small-frame-interval model, not exact finite-rotation image warping.

Covariance is residual-based with a noise floor; it excludes calibration, height and IMU errors.
Dead-reckoning uncertainty is a local approximation, not a correlated visual-inertial EKF.
Unobserved quantities retain large variance. Odometry Z assumes optical height minus rotated
camera offset; it is not an altitude measurement. Height PID cannot verify physical altitude hold
from that constant alone. Gains are not flight-tuned. A powerless glider cannot hold height
indefinitely in still air. target_pose does not arm or drive a vehicle. The Sprint 2 sensor bench
does not close the flight-control loop.

## Verification scope

The GTest suite has 18 mathematical/tracking cases and one ROS-executor case covering both modes,
fallback intrinsics, CameraInfo override, velocity, odometry, LOS, TF and stale magnetic inputs.
Node factories in nodes.hpp allow the identical implementations to run in a single executor with
intra-process communication. This test verifies ROS callbacks and message contracts independently
of WSL's inter-process DDS transport. It does not certify network delivery, Gazebo or flight behavior.
Before simulator integration, verify inter-process DDS delivery separately: discovery alone is
insufficient. No system-wide middleware settings are changed by this package.

## Build and test

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /home/nikol/autopilot/log_visual_nav build \
  --base-paths /home/nikol/autopilot/src/glider_visual_nav \
  --build-base /home/nikol/autopilot/build_visual_nav \
  --install-base /home/nikol/autopilot/install_visual_nav \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon --log-base /home/nikol/autopilot/log_visual_nav test \
  --base-paths /home/nikol/autopilot/src/glider_visual_nav \
  --build-base /home/nikol/autopilot/build_visual_nav \
  --install-base /home/nikol/autopilot/install_visual_nav --packages-select glider_visual_nav
colcon test-result --test-result-base /home/nikol/autopilot/build_visual_nav --verbose
source /home/nikol/autopilot/install_visual_nav/setup.bash
ros2 launch glider_visual_nav navigation.launch.py orientation_mode:=homography_mag
ros2 launch glider_visual_nav navigation.launch.py orientation_mode:=imu_aided
```

## Gazebo Harmonic sensor bench (Sprint 2)

```bash
source /opt/ros/jazzy/setup.bash
source /home/nikol/autopilot/install_visual_nav/setup.bash
ros2 launch glider_visual_nav simulation.launch.py repo_root:=/home/nikol/autopilot
```

This starts Gazebo server with software headless rendering, ros_gz_bridge and three independent
navigation processes. The world holds the Cessna static at optical height 40 m. The model itself
retains rc_cessna's flight dynamics; the world's static override is for sensor qualification.
The camera is mono8, 640x480, 30 Hz, HFOV 1.74 rad, zero distortion, offset [0,0,-0.25] m.
Gazebo CameraInfo supplies the actual K (approximately fx=fy=269.968, cx=320, cy=240).
The 4096-square seeded grayscale ground texture has 1 m tiles and is generated locally by
tools/generate_sim_assets.py. Asset paths live inside the Tools/simulation/gz Git submodule.

All package launch files set the FastDDS profile before starting children. It disables SHM/data sharing,
restricts UDPv4 to 127.0.0.1 and limits RTPS datagrams to 1200 bytes. Discovery uses explicit
loopback peers. SYSTEM_DEFAULT preserves XML settings; Jazzy LOCALHOST would add another UDP
transport and SHM, and SUBNET adds a multicast peer. No machine-wide network settings change.
External ROS tools must use the same profile and domain BEFORE initialization:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nikol/autopilot/src/glider_visual_nav/config/fastdds_wsl.xml
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
```

The world defines ENU B=[4.96673326988e-6, 2.4501664446e-5, -42e-6] T, D=+0.2 rad east.
FixedMagneticFieldSystem uses gz-sensors' physical magnetometer backend at 100 Hz and honors this
world vector. The native Harmonic system replaces it with geographic lookup values, so it is
intentionally absent from this world. ROS receives physical FLU/Tesla; PX4 converts by
1e4*diag(1,-1,-1) to FRD/Gauss. The ideal bench has no magnetic noise; existing native PX4 models
use 3e-7 T standard deviation after the server's legacy unit/frame options are disabled.
Physics steps are 4 ms, so sensor stamp intervals are quantized around nominal 30/100 Hz.

The opt-in evaluator runs all processes, checks sensors and optical odometry, changes only the
bench model's pose, checks four headings and the 30/40/60 m texture, and stops its own children:

```bash
python3 /home/nikol/autopilot/src/glider_visual_nav/test/test_simulation_bench.py \
  --output /home/nikol/autopilot/log_visual_nav/simulation_bench.json
# Optional: also read actual uORB sensor_mag from the rebuilt PX4 (instance 7).
python3 /home/nikol/autopilot/src/glider_visual_nav/test/test_simulation_bench.py --px4 \
  --output /home/nikol/autopilot/log_visual_nav/simulation_px4_bench.json
python3 /home/nikol/autopilot/src/glider_visual_nav/tools/diagnose_wsl_udp.py \
  --output /home/nikol/autopilot/log_visual_nav/udp_mtu_diagnosis.json
```

The evaluator reserves ROS domain 65, a unique GZ_PARTITION, and optional PX4 instance 7. Run one
evaluator at a time. PX4's test startup starts only gz_bridge, with an isolated working directory;
no EKF, GPS or controller feeds navigation. The DDS CTest runs three real-process image/mag
transfers on domains 80–82; the optional diagnosis matrix uses domains 20–24. Best-effort QoS
permits loss under load; the test requires >=70 complete images and >=200 mag samples per run.
See SPRINT2_REPORT.md for measured results and the remaining flight-validation boundary.

## Closed-loop PX4 flight (Sprint 3)

GPS-denied airframe 4022 is registered in PX4. `flight.launch.py` uses the dynamic world,
the pinned Micro XRCE-DDS Agent, exact-checkout px4_msgs, sensor bridges, imu_aided navigation,
LOS and px4_interface_node. It automatically requests attitude Offboard and normal arming.
VehicleAttitudeSetpoint uses `/fmu/in/vehicle_attitude_setpoint_v1`; ROS ENU/FLU is converted
by R_NED,FRD=T*R_WB*S. IMU tilt/gyro and measured barometric height come from PX4, with timestamp
sync disabled so acquisition stamps share Gazebo simulation time. No PX4 position/groundspeed
or Gazebo truth is used by navigation.

Prepare dependencies locally, without changing global ROS installations:

```bash
bash /home/nikol/autopilot/src/glider_visual_nav/tools/prepare_px4_dependencies.sh
source /opt/ros/jazzy/setup.bash
source /home/nikol/autopilot/install_visual_nav/px4_msgs/local_setup.bash
ninja -C /home/nikol/autopilot/build/px4_sitl_default -j4 px4
# Run the package-isolated colcon build/test commands above, then:
source /home/nikol/autopilot/install_visual_nav/setup.bash
python3 /home/nikol/autopilot/src/glider_visual_nav/tools/flight_evaluator.py --duration 45 \
  --output /home/nikol/autopilot/log_visual_nav/cruise_repeat.json
python3 /home/nikol/autopilot/src/glider_visual_nav/tools/flight_evaluator.py --regime unpowered_glide \
  --initial-height 100 --duration 30 --output /home/nikol/autopilot/log_visual_nav/glide_repeat.json
```

`flight_regime` selects `cruise_const_alt` (optical height 40 m, visual speed PI for nominal
12 m/s cruise) or `unpowered_glide` (strict zero thrust and pitch PI for 8.5 m/s). The latter
continuously scales flow using measured h(t). Changing regime requires a coordinated restart.
The barometric ground reference is 0 m for this flat sea-level simulation; it must be calibrated
for a different world. The optical-centre lever arm is included when commanding body height.

The test fixture holds the aircraft until the evaluator releases it with a one-time airlaunch.
After release only normal physics and PX4 actuators move it. Ground truth is read only by the
evaluator; predefined polarity steps run in a separate process without any truth input.
Run trials sequentially: ROS domain 66, Agent port 8888 and PX4 instance 0 are reserved.
Each run saves strict JSON, JSONL, process logs and the finished PX4 ULog into log_visual_nav.
The `--polarity --duration 10` test checks actual roll/pitch signs. The
`--drop-visual-at 8 --duration 13` test terminates its own optical-flow child and checks the
latched wings-level fallback. Full IMU/baro/status loss stops heartbeat and leaves PX4's
configured native Offboard-loss watchdog responsible.

For manual launch, use `ros2 launch glider_visual_nav flight.launch.py flight_regime:=cruise_const_alt`.
The aircraft remains held after ACTIVE until `/evaluation/launch` receives a Vector3d velocity;
the evaluator handles this automatically. This is an airlaunch qualification, not a runway
takeoff or landing demonstration.

See SPRINT3_REPORT.md for measured flight metrics, failed tuning trials, full reproduction
commands, plots and limitations. Final tests include 20 mathematical GTests, one ROS-node test
and three separate-process DDS transfers after an application-level readiness handshake.
