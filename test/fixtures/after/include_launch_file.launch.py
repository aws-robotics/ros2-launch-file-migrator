import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'turtlebot3_bringup'), 'launch/turtlebot3_robot.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'cloudwatch_robot'), 'launch/await_commands.launch.py')
            ),
            launch_arguments={
                'node_name': 'await_commands',
                'type': os.environ.get('AWAIT_TYPE'),
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
