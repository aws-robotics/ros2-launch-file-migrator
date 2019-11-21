import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='node_name',
            default_value='video_encoder'
        ),
        launch.actions.DeclareLaunchArgument(
            name='follow_route',
            default_value='true'
        ),
        launch_ros.actions.Node(
            package='aws_robomaker_simulation_common',
            node_executable='route_manager',
            node_name='route_manager',
            output='screen',
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('follow_route'))
        ),
        launch_ros.actions.Node(
            package='h264_video_encoder',
            node_executable='h264_video_encoder',
            node_name=launch.substitutions.LaunchConfiguration('node_name'),
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('follow_route'))
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
