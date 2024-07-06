import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            os.path.join(get_package_share_directory('freenove_4wd_ros'), 'config', 'drive_config.yaml')],
        ),

        launch_ros.actions.Node(
            package='freenove_4wd_ros', executable='driver',
            name='freenove_driver',
            output='screen',
            parameters=[config_filepath],
        ),
    ])