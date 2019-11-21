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

import unittest

import autopep8

from ros2_launch_file_migrator.launch_actions.launch_action import LaunchAction


class TestLaunchAction(unittest.TestCase):
    def test_serialize(self):
        launch_action = LaunchAction("Node", {
            "package": "'my_package'",
            "node_executable": "'the_type'",
            "node_name": "'the_node'",
            "output": "'log'"
        })
        expected_serialized = autopep8.fix_code("""launch_ros.actions.Node(
            package='my_package',
            node_executable='the_type',
            node_name='the_node',
            output='log'
        )""").rstrip()
        self.assertEqual(expected_serialized, launch_action.serialize())


if __name__ == "__main__":
    unittest.main()
