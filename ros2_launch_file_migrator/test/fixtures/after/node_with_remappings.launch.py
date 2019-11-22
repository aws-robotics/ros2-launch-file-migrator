import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='voice_interaction_robot',
            node_executable='voice_interaction',
            node_name='voice_interaction',
            remappings={
                '/voice_interaction_node/text_output': '/text_output',
                '/voice_interaction_node/audio_output': '/audio_output'
            }
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
