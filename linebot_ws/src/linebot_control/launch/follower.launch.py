import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('linebot_control'), 'config', 'follower_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(package='linebot_control', executable='line_detector', name='line_detector',
             output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(package='linebot_control', executable='line_follower', name='line_follower',
             output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}]),
    ])
