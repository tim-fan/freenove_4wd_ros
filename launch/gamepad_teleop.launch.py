import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    drive_config_filepath = launch.substitutions.LaunchConfiguration('drive_config_filepath')


    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument('drive_config_filepath', default_value=[
                os.path.join(get_package_share_directory('freenove_4wd_ros'), 'config', 'drive_config.yaml')],
            ),

            # launch freenove driver
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('freenove_4wd_ros'), 'launch'), '/driver.launch.py']
                ),
                launch_arguments={
                    'config_filepath':drive_config_filepath,
                }.items(),
            ),

            # launch joy teleop
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('teleop_twist_joy'), 'launch'), '/teleop-launch.py']
                ),
                launch_arguments={
                    'config_filepath':os.path.join(
                        get_package_share_directory('freenove_4wd_ros'), 'config', 'gamepad_config.yaml'
                    ),
                }.items(),
            )
        ]
    )