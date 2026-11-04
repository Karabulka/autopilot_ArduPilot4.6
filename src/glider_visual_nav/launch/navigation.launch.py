"""Navigation nodes using the same WSL DDS profile as the simulation bench."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('glider_visual_nav'), 'config', 'navigation.yaml')
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', os.path.join(os.path.dirname(config), 'fastdds_wsl.xml')),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', os.path.join(os.path.dirname(config), 'fastdds_wsl.xml')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SYSTEM_DEFAULT'),
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('orientation_mode', default_value='homography_mag',
                              choices=['homography_mag', 'imu_aided']),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *[Node(package='glider_visual_nav', executable=name, name=name, output='screen',
               parameters=[LaunchConfiguration('params_file'), {
                   'orientation_mode': LaunchConfiguration('orientation_mode'),
                   'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
               }]) for name in ['optical_flow_node', 'navigation_node', 'los_controller_node']],
    ])
