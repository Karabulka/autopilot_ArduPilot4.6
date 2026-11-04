# Source this file in every terminal used for the demonstration.
export GLIDER_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$GLIDER_WORKSPACE/install_visual_nav/setup.bash"
export ROS_DOMAIN_ID=66
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$GLIDER_WORKSPACE/src/glider_visual_nav/config/fastdds_wsl.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export ROS_LOCALHOST_ONLY=0
export GZ_PARTITION=glider_demo
export GZ_IP=127.0.0.1
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
export GZ_SIM_RESOURCE_PATH="$GLIDER_WORKSPACE/src/glider_visual_nav/simulation/models:$GLIDER_WORKSPACE/Tools/simulation/gz/models:$GLIDER_WORKSPACE/Tools/simulation/gz/worlds"
