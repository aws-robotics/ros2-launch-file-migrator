import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='aws_robomaker_simulation_common',
            node_executable='route_manager',
            node_name='route_manager',
            output='screen',
            parameters=[
                get_package_share_directory(
                    'book_world') + '/routes/route.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
