"""GPS-denied PX4 attitude Offboard, sensor navigation and isolated truth evaluator topic."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('glider_visual_nav')
    config = os.path.join(share, 'config')
    root = LaunchConfiguration('repo_root')
    regime = LaunchConfiguration('flight_regime')
    def prepare(context):
        directory=Path(LaunchConfiguration('px4_workdir').perform(context))
        directory.mkdir(parents=True,exist_ok=True)
        expected=Path(root.perform(context))/'build/px4_sitl_default/etc'
        link=directory/'etc'
        if link.is_symlink() and link.resolve()!=expected.resolve():
            link.unlink()
        if not link.exists(): link.symlink_to(expected)
        elif link.resolve()!=expected.resolve(): raise RuntimeError('PX4 workdir contains an unrelated etc directory')
        return []
    return LaunchDescription([
        DeclareLaunchArgument('repo_root', default_value='/home/nikol/autopilot'),
        DeclareLaunchArgument('flight_regime', default_value='cruise_const_alt', choices=['cruise_const_alt', 'unpowered_glide']),
        DeclareLaunchArgument('guidance_enabled', default_value='true'),
        DeclareLaunchArgument('glide_speed', default_value='8.5'),
        DeclareLaunchArgument('world_file', default_value=PathJoinSubstitution([root, 'Tools/simulation/gz/worlds/visual_nav_flight.sdf'])),
        DeclareLaunchArgument('px4_workdir', default_value=PathJoinSubstitution([root, 'build_visual_nav/flight_px4'])),
        OpaqueFunction(function=prepare),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', os.path.join(config, 'fastdds_wsl.xml')),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', os.path.join(config, 'fastdds_wsl.xml')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SYSTEM_DEFAULT'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [root, '/Tools/simulation/gz/models:', root, '/Tools/simulation/gz/worlds']),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', get_package_prefix('glider_visual_nav') + '/lib'),
        SetEnvironmentVariable('LD_LIBRARY_PATH', [root, '/install_visual_nav/xrce_agent/lib:', os.environ.get('LD_LIBRARY_PATH', '')]),
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('PX4_SYS_AUTOSTART', '4022'),
        SetEnvironmentVariable('PX4_GZ_STANDALONE', '1'),
        SetEnvironmentVariable('PX4_GZ_WORLD', 'visual_nav_flight'),
        SetEnvironmentVariable('PX4_GZ_MODEL_NAME', 'rc_cessna_visual'),
        SetEnvironmentVariable('UXRCE_DDS_AG_IP', '127.0.0.1'),
        SetEnvironmentVariable('HEADLESS', '1'),
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', LaunchConfiguration('world_file')], output='screen'),
        ExecuteProcess(cmd=[PathJoinSubstitution([root, 'install_visual_nav/xrce_agent/bin/MicroXRCEAgent']), 'udp4', '-p', '8888'], output='screen'),
        ExecuteProcess(cmd=[PathJoinSubstitution([root, 'build/px4_sitl_default/bin/px4']), '-d', '-i', '0', '-w',
                            LaunchConfiguration('px4_workdir'), '-s', PathJoinSubstitution([root, 'build/px4_sitl_default/etc/init.d-posix/rcS']),
                            PathJoinSubstitution([root, 'build/px4_sitl_default/etc'])], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='flight_sensor_bridge', output='screen',
             parameters=[{'config_file': os.path.join(config, 'flight_bridge.yaml'), 'use_sim_time': True}]),
        *[Node(package='glider_visual_nav', executable=name, name=name, output='screen',
               parameters=[os.path.join(config, 'navigation.yaml'), os.path.join(config, 'flight.yaml'), {
                   'flight_regime': regime,
                   'glide_speed': ParameterValue(LaunchConfiguration('glide_speed'),value_type=float)}],
               condition=IfCondition(LaunchConfiguration('guidance_enabled')) if name == 'los_controller_node' else None)
          for name in ['optical_flow_node', 'navigation_node', 'los_controller_node', 'px4_interface_node']],
    ])
