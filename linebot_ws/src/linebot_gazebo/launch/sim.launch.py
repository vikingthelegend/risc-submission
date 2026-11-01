"""Gazebo Harmonic + robot spawn + ROS bridge (no follower).

ros2 launch linebot_gazebo sim.launch.py track:=oval
ros2 launch linebot_gazebo sim.launch.py world:=/abs/path/my_world.sdf x:=0 y:=0 yaw:=1.57
ros2 launch linebot_gazebo sim.launch.py gui:=false headless:=true
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_NAME = 'line_track'      # must match <world name=...> in the SDF


def _setup(context, *args, **kwargs):
    gz_share = get_package_share_directory('linebot_gazebo')
    desc_share = get_package_share_directory('linebot_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    track = LaunchConfiguration('track').perform(context)
    world = LaunchConfiguration('world').perform(context) or \
        os.path.join(gz_share, 'worlds', f'{track}.sdf')
    if not os.path.isfile(world):
        raise RuntimeError(f'World file not found: {world}')

    # start pose: explicit args win, otherwise config/start_poses.yaml[track]
    with open(os.path.join(gz_share, 'config', 'start_poses.yaml')) as f:
        pose = (yaml.safe_load(f) or {}).get(track, {})
    x = LaunchConfiguration('x').perform(context) or str(pose.get('x', 0.0))
    y = LaunchConfiguration('y').perform(context) or str(pose.get('y', 0.0))
    yaw = LaunchConfiguration('yaw').perform(context) or str(pose.get('yaw', 0.0))

    gz_args = ['-r']
    if LaunchConfiguration('gui').perform(context).lower() in ('false', '0'):
        gz_args.append('-s')
    if LaunchConfiguration('headless').perform(context).lower() in ('true', '1'):
        gz_args.append('--headless-rendering')
    gz_args.append(world)

    robot_description = xacro.process_file(
        os.path.join(desc_share, 'urdf', 'linebot.urdf.xacro')).toxml()

    return [
        # lets worlds reference textures/meshes relative to worlds/
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(gz_share, 'worlds')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ' '.join(gz_args), 'on_exit_shutdown': 'true'}.items()),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', WORLD_NAME, '-name', 'linebot', '-topic', 'robot_description',
                        '-x', x, '-y', y, '-z', '0.05', '-Y', yaw]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(gz_share, 'config', 'bridge.yaml'),
                          'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('track', default_value='oval',
                              description='worlds/<track>.sdf: straight circle oval s_curve '
                                          'square_loop figure_eight, or one you generated'),
        DeclareLaunchArgument('world', default_value='', description='absolute path to a custom SDF world'),
        DeclareLaunchArgument('x', default_value='', description='spawn x (default: start_poses.yaml)'),
        DeclareLaunchArgument('y', default_value='', description='spawn y'),
        DeclareLaunchArgument('yaw', default_value='', description='spawn yaw [rad]'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='use --headless-rendering (for servers without a display)'),
        OpaqueFunction(function=_setup),
    ])
