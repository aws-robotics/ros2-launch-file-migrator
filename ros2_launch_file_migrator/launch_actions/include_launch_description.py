# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.

import autopep8
import os

from jinja2 import Environment, FileSystemLoader

from .launch_action import LaunchAction

class IncludeLaunchDescription(LaunchAction):
    """An IncludeLaunchDescription launch action

    This class is used for defining IncludeLaunchDescription launch actions
    which include another ROS2 launch file into this one. 
    """

    def __init__(self, package, launch_file_path, arguments):
        self.action_type = "IncludeLaunchDescription"
        self.package = package
        self.launch_file_path = launch_file_path
        self.arguments = arguments

    def serialize(self):
        """Serialize the launch action as a pep8 formatted string"""

        src_dir = os.path.dirname(__file__)
        templates_path = os.path.join(src_dir, '..', 'templates')
        env = Environment(
            loader=FileSystemLoader(searchpath=templates_path)
        )
        template = env.get_template('include_launch_description.j2')
        launch_file_path = self.get_formatted_launch_file_path()
        launch_arguments = self.get_formatted_arguments()
        rendered_template = template.render(
            package=self.package,
            launch_file_path=launch_file_path,
            launch_arguments=launch_arguments
        )
        return autopep8.fix_code(rendered_template).rstrip()

    def get_formatted_launch_file_path(self):
        """Formats the path to the launch file adding .py to the end if required"""
        launch_file_path = self.launch_file_path.lstrip("/")
        if not launch_file_path.endswith(".py"):
            launch_file_path += ".py"
        return launch_file_path

    def get_formatted_arguments(self):
        """Formats all launch file arguments into a newline separated string
        
        This format is so the arguments can be inserted inside a dictionary in
        the include_launch_description template
        """
        if len(self.arguments) == 0:
            return None
        formatted_arguments = []
        for arg, value in self.arguments.items():
            formatted_arguments.append(f"'{arg}': {value}")
        return ",\n".join(formatted_arguments)

