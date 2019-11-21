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

import os
import unittest

from ros2_launch_file_migrator import LaunchFileMigrator


class EndToEndTests(unittest.TestCase):
    def compare_complete_file_conversion(self, file_name):
        self.maxDiff = None
        test_dir = os.path.dirname(__file__)
        input_launch_file = f"{test_dir}/fixtures/before/{file_name}.launch"
        expected_launch_file = f"{test_dir}/fixtures/after/{file_name}.launch.py"
        with open(input_launch_file, "r") as file_handle:
            input_file_contents = file_handle.read()
        with open(expected_launch_file, "r") as file_handle:
            expected_file_contents = file_handle.read()
        self.assertEqual(expected_file_contents,
                         LaunchFileMigrator.migrate(input_file_contents))

    def test_single_node(self):
        self.compare_complete_file_conversion("single_node")

    def test_node_with_args(self):
        self.compare_complete_file_conversion("node_with_args")

    def test_node_with_params(self):
        self.compare_complete_file_conversion("node_with_params")

    def test_nodes_with_global_param(self):
        self.compare_complete_file_conversion("nodes_with_global_param")

    def test_include_launch_file(self):
        self.compare_complete_file_conversion("include_launch_file")

    def test_group_with_nodes(self):
        self.compare_complete_file_conversion("group_with_nodes")

    def test_node_with_rosparam(self):
        self.compare_complete_file_conversion("node_with_rosparam")

    def test_node_with_remappings(self):
        self.compare_complete_file_conversion("node_with_remappings")


if __name__ == "__main__":
    unittest.main()
