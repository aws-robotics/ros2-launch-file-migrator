import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='node_name',
            default_value='h264_video_encoder'
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi'),
            description='model type [burger, waffle, waffle_pi]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='output',
            default_value='log',
            description="The stdout/stderr location for this node. Set to 'screen' to see this node's output in the terminal."
        ),
        launch_ros.actions.Node(
            package='h264_video_encoder',
            node_executable='h264_video_encoder',
            node_name=launch.substitutions.LaunchConfiguration('node_name'),
            output=launch.substitutions.LaunchConfiguration('output')
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
