# Sprint 2 verification — 2026-10-20

## Implemented scope

The headless Gazebo Harmonic bench, ros_gz_bridge, optical_flow_node, navigation_node and
los_controller_node run as distinct OS processes. Navigation receives only camera and magnetic
measurements plus clock. The evaluator alone changes model pose. The model is static in this
qualification world; this report does not claim autonomous flight or actuator-loop validation.

## DDS diagnosis and correction

Environment: Ubuntu 24.04 WSL2, Windows networkingMode=mirrored, firewall=true, ROS 2 Jazzy,
rmw_fastrtps 8.4.3, Fast DDS 2.14.5. lo advertises MTU 65536 while eth3/eth4 advertise 1500.
No system network, firewall, kernel, or global ROS configuration was changed.

The standalone tools/diagnose_wsl_udp.py launches a separate Python sender and receives on an
ephemeral 127.0.0.1 port, without loading ROS or DDS. Measured delivery:

| UDP payload bytes | Delivered |
| --- | --- |
| 1200, 1400, 1472 | yes |
| 1473, 1500, 2000, 6000, 60000 | no (timeout) |

The boundary is 1500 bytes minus IPv4/UDP headers (20+8). Oversized local UDP datagrams disappear
in the current mirrored-network path, despite the advertised loopback MTU. This is an observed
transport/fragmentation failure independent of SHM or multicast. The experiment does not identify
which Windows/WSL driver/filter drops them. Disabling SHM alone is insufficient.

The controlled DDS matrix changes one key property between its final two cases:

| Profile | Discovery/data |
| --- | --- |
| default FastDDS | failed |
| UDP only, all interfaces | failed |
| UDP only, loopback multicast | failed |
| UDP loopback, explicit unicast peers, maxMessageSize=65000 | failed |
| same profile, maxMessageSize=1200 | passed: 114 complete images / 339 magnetic samples |

The successful matrix publisher sent 116 images; 2 were dropped with best-effort QoS under load.
test_dds_processes runs three independent process pairs, with full 640x480 mono8 payloads,
checksum validation and >=70 images / >=200 field messages required per 4-second publication
period. It uses real cross-process DDS, with no intra-process shortcut or ROS imports in its parent.
The 19 existing GTests and the process test pass. Colcon reports 23 aggregated test entries,
0 errors / 0 failures (including the ament/CTest result wrappers).

config/fastdds_wsl.xml disables builtin transports and data sharing, enables UDPv4 on 127.0.0.1,
uses explicit loopback discovery peers (up to 32 participants), and limits RTPS messages to 1200
bytes. Payload fragmentation therefore happens inside RTPS rather than relying on oversized IP
datagrams. launch files apply the profile to every ROS child before initialization. Discovery range
SYSTEM_DEFAULT preserves these XML choices: Jazzy's LOCALHOST option injects additional UDP/SHM
transports, and SUBNET adds a multicast announcement peer.

Reproduce with the commands in README.md and test/test_dds_processes.py --diagnose. Raw evidence:
/home/nikol/autopilot/log_visual_nav/{udp_mtu_diagnosis,dds_diagnosis}.json and the isolated CTest
results under /home/nikol/autopilot/build_visual_nav/glider_visual_nav/test_results.

## Magnetic contract

World W is ENU with true north +Y. East-positive declination D=0.2 rad, horizontal field 25 uT,
vertical field -42 uT. B_W=[4.96673326988e-6,2.4501664446e-5,-42e-6] T.
For pose R_WB, the sensor publishes B_B=R_WB^T B_W in physical FLU/Tesla.
PX4 sensor_mag=1e4 diag(1,-1,-1) B_B in FRD/Gauss. This rotation has determinant +1.
ROS keeps B_B unchanged. navigation_node uses the same D to recover ENU yaw.

The installed native Gazebo magnetometer system recomputes world field from geographic lookup
tables. FixedMagneticFieldSystem instead supplies the explicit SDF vector to the gz-sensors
magnetometer backend, preserving its sensor pose, timestamp and noise handling. This world loads
only the fixed-field system. Native PX4 server configs disable legacy Gauss/NED options, and the
12 pre-existing models' magnetic noise standard deviations are changed from 0.0001 to 3e-7 T.
The new ideal bench model has no magnetic noise. These are intentional global corrections required
by the matching change in GZBridge.cpp; using the old server configuration with the new callback
would violate the contract.

Both modules__simulation__gz_bridge and the complete px4 SITL executable were rebuilt. An isolated
PX4 instance 7 started only gz_bridge, and the evaluator read actual uORB sensor_mag at every yaw:

| ENU yaw | ROS field in uT (FLU) | PX4 field in Gauss (FRD, listener precision) |
| --- | --- | --- |
| 0 degrees | [4.9667,24.5017,-42] | [0.04967,-0.24502,0.42000] |
| 90 degrees | [24.5017,-4.9667,-42] | [0.24502,0.04967,0.42000] |
| 180 degrees | [-4.9667,-24.5017,-42] | [-0.04967,0.24502,0.42000] |
| 270 degrees | [-24.5017,4.9667,-42] | [-0.24502,-0.04967,0.42000] |

Maximum printed PX4 comparison error is 3.36e-6 Gauss, within the listener's five-decimal rounding.
This verifies the actual bridge callback; it does not claim PX4 EKF heading convergence.

## Camera, texture and end-to-end measurements

rc_cessna_visual derives from rc_cessna with its dynamics, mesh URIs and actuator plugins retained.
NavSat is removed. The nadir camera is placed 0.25 m below base_link, with Gazebo sensor +X rotated
by pitch +pi/2 to point down. Gazebo's rendering-to-optical convention yields
R_BC=[[0,-1,0],[-1,0,0],[0,0,-1]] exactly: image top is the nose. No reflection TF is introduced.

Camera: mono8, 640x480, 30 Hz, horizontal FOV 1.74 rad, pinhole/no distortion. Actual CameraInfo:
fx=269.96816635, fy=269.96818542, cx=320, cy=240. The simulation fallback uses the matching K;
CameraInfo takes precedence. The world starts base_link at 40.25 m, hence optical height h=40 m.

Ground: 512x512 m plane, seeded 4096x4096 grayscale albedo texture with 1 m random tiles,
zero wind and fixed lighting. Texture generation is deterministic and uses no remote assets.
At each measured central pose, all sampled images gave 500 Shi-Tomasi corners (the configured
maximum). optical_flow_node diagnostics independently showed 500 LK tracks / 500 RANSAC inliers.

| Camera height | ENU yaw | Corners per sampled image | Stationary speed |
| --- | --- | --- | --- |
| 40 m | 0,90,180,270 degrees | 500 | <1e-12 m/s |
| 30 m | 0 degrees | 500 | <1e-12 m/s |
| 60 m | 0 degrees | 500 | <1e-12 m/s |

Recovered navigation yaw error was below 1e-8 degree for all four headings in the ideal bench.
This precision reflects deterministic noise-free measurements. Teleport transitions are excluded
by a settling interval and can legitimately reject one image pair. Guidance withholds commands
at near-zero airframe speed. Physics uses 4 ms steps, so 30/100 Hz acquisition timestamps are
quantized; wall-clock performance also depends on software rendering and CPU load.

Raw sensor/odometry results, counts, camera K and per-pose PX4 values are recorded in
/home/nikol/autopilot/log_visual_nav/simulation_px4_bench.json, with process logs alongside it.
The final evaluator received 645 images, 639 CameraInfo, 2039 magnetic messages and 662 odometry
messages. Effective received rates over simulated time were 28.87 Hz (image) and 91.28 Hz (mag),
including startup and best-effort losses under load. The configured sensor rates are 30/100 Hz;
this run does not claim lossless delivery at those rates.
camera_h*_yaw*.png files show the actual rendered images. test/test_simulation_bench.py --px4
reproduces the test, checks numeric tolerances, saves its report even on failure, and stops its
own launch/PX4 processes.

## Changed file groups and integration boundary

- Root AGENTS.md: field, units and WSL transport contracts.
- src/glider_visual_nav/config/{fastdds_wsl.xml,bridge.yaml,simulation.yaml}: DDS and sensor bridges.
- src/glider_visual_nav/launch/{navigation.launch.py,simulation.launch.py}: process environment/startup.
- src/glider_visual_nav/src/fixed_magnetic_field_system.cpp: deterministic physical magnetometer.
- src/glider_visual_nav/src/optical_flow_node.cpp: throttled track/inlier/velocity diagnostics.
- src/glider_visual_nav/test/{dds_peer.cpp,test_dds_processes.py,test_simulation_bench.py}: real-process tests.
- src/glider_visual_nav/tools/{generate_sim_assets.py,diagnose_wsl_udp.py}: reproducible assets/diagnosis.
- src/glider_visual_nav/{CMakeLists.txt,package.xml,README.md}: dependencies, builds and instructions.
- src/modules/simulation/gz_bridge/{GZBridge.cpp,server.config}: proper rotation and units.
- Tools/simulation/gz/{worlds/visual_nav_flat.sdf,models/rc_cessna_visual,models/visual_nav_ground}:
  qualification world, camera airframe and local texture assets.
- Tools/simulation/gz/server.config and 12 existing model.sdf files: matching native mag units/noise.

Gazebo assets and noise changes are uncommitted changes inside the existing Git submodule, whose
base commit remains unchanged. Parent and submodule changes must be versioned together when
publishing. No unrelated pre-existing user changes were reverted.

The bench establishes sensor delivery, calibration, frame/field agreement and stationary odometry.
It does not validate moving-aircraft scale, banked flight, altitude observability, actuator mappings,
flight-tuned gains, arming or closed-loop route tracking. Constant height remains an assumption.
The simulation launch uses homography_mag; imu_aided still requires its specified body-frame IMU
contract and is not enabled by this camera/magnetometer qualification world.

## Primary implementation references

- Gazebo 8.11 magnetometer: https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/systems/magnetometer/Magnetometer.cc
- Jazzy rmw_fastrtps 8.4.3 discovery overrides: https://github.com/ros2/rmw_fastrtps/blob/8.4.3/rmw_fastrtps_shared_cpp/src/participant.cpp
- FastDDS XML transport/discovery: https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/xml_configuration/domainparticipant.html
