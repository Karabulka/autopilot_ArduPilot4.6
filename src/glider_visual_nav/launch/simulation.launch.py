"""Independent Gazebo, bridge and navigation processes; fixed-field sensor bench."""
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('glider_visual_nav')
    config = os.path.join(share, 'config')
    root = LaunchConfiguration('repo_root')
    return LaunchDescription([
        DeclareLaunchArgument('repo_root', default_value='/home/nikol/autopilot'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', os.path.join(config, 'fastdds_wsl.xml')),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', os.path.join(config, 'fastdds_wsl.xml')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SYSTEM_DEFAULT'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [root, '/Tools/simulation/gz/models:', root, '/Tools/simulation/gz/worlds']),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', get_package_prefix('glider_visual_nav') + '/lib'),
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '--headless-rendering',
                            PathJoinSubstitution([root, 'Tools/simulation/gz/worlds/visual_nav_flat.sdf'])], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sensor_bridge', output='screen',
             parameters=[{'config_file': os.path.join(config, 'bridge.yaml'), 'use_sim_time': True}]),
        *[Node(package='glider_visual_nav', executable=name, name=name, output='screen',
               parameters=[os.path.join(config, 'navigation.yaml'), os.path.join(config, 'simulation.yaml')])
          for name in ['optical_flow_node', 'navigation_node', 'los_controller_node']],
    ])
