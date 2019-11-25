import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        launch_ros.actions.Node(
            package='python_launcher',
            node_executable='run_consume_memory.sh',
            node_name='consume_memory',
            output='screen',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='python_launcher',
            node_executable='run_cpu_check.sh',
            node_name='cpu_check',
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
