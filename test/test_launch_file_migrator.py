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

import xml.etree.ElementTree as etree

from ros2_launch_file_migrator import LaunchFileMigrator
from ros2_launch_file_migrator.launch_actions.launch_action import LaunchAction


class TestLaunchFileMigrator(unittest.TestCase):
    def test_convert_node_elem(self):
        launch_file_xml = ("<launch>\n"
                           '<node pkg="my_package" type="the_type" name="the_node" output="screen" />\n'
                           '</launch>')
        expected_action = LaunchAction("Node", {
            "package": "'my_package'",
            "node_executable": "'the_type'",
            "node_name": "'the_node'",
            "output": "'screen'"
        })
        migrator = LaunchFileMigrator()
        xml_tree = etree.fromstring(launch_file_xml)
        self.assertEqual(migrator.convert_node_elem(
            xml_tree[0]), expected_action)

    def test_convert_attribute_simple_string(self):
        node_parameter = "my_node"
        expected_result = "'my_node'"
        migrator = LaunchFileMigrator()
        self.assertEqual(
            expected_result, migrator.convert_attribute(node_parameter))

    def test_convert_attribute_string_with_apostraphe(self):
        node_parameter = "This isn't a description"
        expected_result = "\"This isn't a description\""
        migrator = LaunchFileMigrator()
        self.assertEqual(
            expected_result, migrator.convert_attribute(node_parameter))

    def test_convert_attribute_is_arg(self):
        migrator = LaunchFileMigrator()
        node_parameter = "$(arg node_name)"
        expected_result = "launch.substitutions.LaunchConfiguration('node_name')"
        self.assertEqual(
            expected_result, migrator.convert_attribute(node_parameter))

    def test_convert_attribute_is_find(self):
        migrator = LaunchFileMigrator()
        node_parameter = "$(find turtlebot3_navigation)/maps/map.yaml"
        expected_result = "get_package_share_directory('turtlebot3_navigation') + '/maps/map.yaml'"
        expected_include = "from ament_index_python.packages import get_package_share_directory"
        self.assertEqual(
            expected_result, migrator.convert_attribute(node_parameter))
        self.assertTrue(expected_include in migrator.imports)

    def test_convert_attribute_is_optenv(self):
        migrator = LaunchFileMigrator()
        node_parameter = "$(optenv LAUNCH_ID my_launch)"
        expected_result = "os.environ.get('LAUNCH_ID', 'my_launch')"
        self.assertEqual(
            expected_result, migrator.convert_attribute(node_parameter))

    def test_convert_attribute_is_env(self):
        migrator = LaunchFileMigrator()
        node_parameter = "$(env LAUNCH_ID)"
        expected_result = "os.environ.get('LAUNCH_ID')"
        self.assertEqual(
            expected_result, migrator.convert_attribute(node_parameter))

    def test_convert_attribute_required(self):
        # Tests that the 'required' node parameter is converted correctly
        launch_file_xml = ('<launch>\n'
                           '<node pkg="my_package" required="true" />\n'
                           '</launch>')
        expected_action = LaunchAction("Node", {
            "package": "'my_package'",
            "on_exit": "launch.actions.Shutdown()"
        })
        migrator = LaunchFileMigrator()
        xml_tree = etree.fromstring(launch_file_xml)
        converted_node = migrator.convert_node_elem(xml_tree[0])
        self.assertEqual(converted_node, expected_action)

    def test_convert_arg_elem_default(self):
        # Tests that the 'default' arg attribute is turned into default_value in ROS2
        launch_file_xml = ('<launch>\n'
                           '<arg name="logger_node_name" default="my_logger" />\n'
                           '</launch>')
        expected_action = LaunchAction("DeclareLaunchArgument", {
            "name": "'logger_node_name'",
            "default_value": "'my_logger'"
        })
        migrator = LaunchFileMigrator()
        xml_tree = etree.fromstring(launch_file_xml)
        converted_node = migrator.convert_arg_elem(xml_tree[0])
        self.assertEqual(converted_node, expected_action)

    def test_convert_arg_elem_value(self):
        # Tests that the 'value' arg attribute is turned into default_value in ROS2
        launch_file_xml = ('<launch>\n'
                           '<arg name="metrics_node_name" value="$(optenv METRICS_NODE_NAME my_node)" />\n'
                           '</launch>')
        expected_action = LaunchAction("DeclareLaunchArgument", {
            "name": "'metrics_node_name'",
            "default_value": "os.environ.get('METRICS_NODE_NAME', 'my_node')"
        })
        migrator = LaunchFileMigrator()
        xml_tree = etree.fromstring(launch_file_xml)
        converted_node = migrator.convert_arg_elem(xml_tree[0])
        self.assertEqual(converted_node, expected_action)

    def test_convert_rosparam_elem(self):
        # Tests that rosparam's loading a file are converted into a file path for ROS2 
        # so that they can be added to a nodes param list
        launch_file_xml = ('<launch>\n'
                           '<rosparam file="$(find book_world)/routes/route.yaml" command="load"/>\n'
                           '</launch>')
        expected_parameters = [
            "get_package_share_directory('book_world') + '/routes/route.yaml'"
        ]
        migrator = LaunchFileMigrator()
        xml_tree = etree.fromstring(launch_file_xml)
        converted_parameters = migrator.convert_rosparam_elems(xml_tree)
        self.assertEqual(converted_parameters, expected_parameters)

    def test_convert_rosparam_elem_with_name(self):
        # Tests that rosparam's with a namespace are skipped
        launch_file_xml = ('<launch>\n'
                           '<rosparam param="my/param/name" file="$(find book_world)/routes/route.yaml" command="load"/>\n'
                           '</launch>')
        expected_parameters = []
        migrator = LaunchFileMigrator()
        xml_tree = etree.fromstring(launch_file_xml)
        converted_parameters = migrator.convert_rosparam_elems(xml_tree)
        self.assertEqual(converted_parameters, expected_parameters)

    def test_convert_rosparam_elem_with_ns(self):
        # Tests that rosparam's with a namespace are skipped
        launch_file_xml = ('<launch>\n'
                           '<rosparam ns="test/namespace" file="$(find book_world)/routes/route.yaml" command="load"/>\n'
                           '</launch>')
        expected_parameters = []
        migrator = LaunchFileMigrator()
        xml_tree = etree.fromstring(launch_file_xml)
        converted_parameters = migrator.convert_rosparam_elems(xml_tree)
        self.assertEqual(converted_parameters, expected_parameters)





if __name__ == "__main__":
    unittest.main()
