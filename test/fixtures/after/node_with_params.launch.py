import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='image_transport',
            default_value='compressed',
            description="The image transport used by this video encoder node. This can be 'raw' or 'compressed'."
        ),
        launch_ros.actions.Node(
            package='h264_video_encoder',
            node_executable='h264_video_encoder',
            node_name='h264_video_encoder',
            parameters=[
                {
                    'image_transport': launch.substitutions.LaunchConfiguration('image_transport')
                },
                {
                    'stream_count': '1'
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
