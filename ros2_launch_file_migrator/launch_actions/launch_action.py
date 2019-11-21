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


class LaunchAction:
    """A single LaunchAction in a launch file.

    This class is useful for defining the actions in a launch file and then 
    serializing them into python code to place into the ROS2 launch file. 
    """

    def __init__(self, action_type, parameters=None):
        self.action_type = action_type
        self.parameters = parameters
        self.condition = None

    def __eq__(self, other):
        return self.serialize() == other.serialize()

    def set_parameter(self, key, value):
        self.parameters[key] = value

    def get_actions_package_name(self):
        if self.action_type == "Node":
            return "launch_ros"
        return "launch"

    def serialize(self):
        """Serialize the launch action as a pep8 formatted string.

        Turn this launch action into a pip8 formatted python function that 
        can be inserted into the launch actions array in the launch file. 
        """
        package_name = self.get_actions_package_name()
        serialized = f"{package_name}.actions.{self.action_type}(\n"
        parameter_strings = []
        if self.parameters != None:
            for name, value in self.parameters.items():
                if value != None:
                    serialized_value = self.serialize_value(value)
                    parameter_strings.append(f"{name}={serialized_value}")
        if self.condition != None:
            parameter_strings.append(f"condition={self.condition.serialize()}")
        serialized += ",\n".join(parameter_strings)
        serialized += "\n)"
        return autopep8.fix_code(serialized).rstrip()

    def serialize_value(self, value):
        """Serialize a single parameter value for a launch action.

        This is used to correct serialize dictionaries and lists as python code.
        As the default f-string print will wrap them in unneccessary quotes
        which doesn't compile correctly
        """
        if isinstance(value, list):
            serialized = "[\n"
            item_strings = []
            for item in value:
                item_strings.append(self.serialize_value(item))
            serialized += ",\n".join(item_strings)
            serialized += "\n]"
            return serialized
        if isinstance(value, dict):
            serialized = "{\n"
            item_strings = []
            for key, val in value.items():
                item_strings.append(f"'{key}':{val}")
            serialized += ",\n".join(item_strings)
            serialized += "\n}"
            return serialized
        return str(value)

    def add_condition(self, condition):
        self.condition = condition

