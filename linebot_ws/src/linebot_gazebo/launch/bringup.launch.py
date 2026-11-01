"""Everything: Gazebo + robot + bridge + line detector + PID follower.

ros2 launch linebot_gazebo bringup.launch.py track:=s_curve
ros2 launch linebot_gazebo bringup.launch.py follow:=false      # drive it by hand
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PASS_THROUGH = ['track', 'world', 'x', 'y', 'yaw', 'gui', 'headless']
DEFAULTS = {'track': 'oval', 'world': '', 'x': '', 'y': '', 'yaw': '', 'gui': 'true',
            'headless': 'false'}


def generate_launch_description():
    gz = get_package_share_directory('linebot_gazebo')
    ctl = get_package_share_directory('linebot_control')

    args = [DeclareLaunchArgument(k, default_value=v) for k, v in DEFAULTS.items()]
    args += [
        DeclareLaunchArgument('follow', default_value='true',
                              description='start detector + PID follower'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(ctl, 'config', 'follower_params.yaml')),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'sim.launch.py')),
        launch_arguments={k: LaunchConfiguration(k) for k in PASS_THROUGH}.items())

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ctl, 'launch', 'follower.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'params_file': LaunchConfiguration('params_file')}.items(),
        condition=IfCondition(LaunchConfiguration('follow')))

    return LaunchDescription(args + [sim, follower])
