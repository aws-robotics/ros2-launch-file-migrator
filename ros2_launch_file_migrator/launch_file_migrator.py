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

import logging
import os
import re

import autopep8
from jinja2 import Environment, FileSystemLoader
import xml.etree.ElementTree as etree

from .launch_actions.launch_action import LaunchAction
from .launch_conditions.launch_condition import LaunchCondition
from .launch_actions.include_launch_description import IncludeLaunchDescription


class LaunchFileMigrator:
    """A class for migrating a launch file from ROS1 to ROS2"""

    def __init__(self):
        self.imports = set()
        self.global_params = []
        self.launch_actions = []

    @staticmethod
    def migrate(launch_file_contents):
        """Migrate a launch file from ROS1 XML to ROS2 python layout

        Arguments:
            launch_file_contents - The contents 
        Returns:
            A string of Python code for the new launch file
        """
        migrator = LaunchFileMigrator()
        converted_code = migrator.migrate_launch_file(launch_file_contents)
        return converted_code

    def migrate_launch_file(self, launch_file_contents):
        """Migrate a launch file from ROS1 XML to ROS2 python layout

        Arguments:
            launch_file_contents - The contents 
        Returns:
            A string of Python code for the new launch file
        """
        self.xml_tree_root = etree.fromstring(launch_file_contents)
        self.convert_args()
        self.gather_global_params()
        self.launch_actions += self.convert_groups()
        self.launch_actions += self.convert_nodes(self.xml_tree_root)
        self.launch_actions += self.convert_includes()
        return self.generate_launch_file()

    def generate_launch_file(self):
        """Return a pep8 formatted ROS2 launch file with defined imports and actions"""
        src_dir = os.path.dirname(__file__)
        env = Environment(
            loader=FileSystemLoader(searchpath=src_dir + '/templates')
        )
        launch_file_imports = self.generate_imports()
        launch_file_actions = self.generate_actions()
        template = env.get_template('launch_file.j2')
        rendered_template = template.render(
            imports=launch_file_imports,
            actions=launch_file_actions
        )
        return autopep8.fix_code(rendered_template)

    def generate_imports(self):
        """Return a string of newline separated python imports"""
        return "\n".join(self.imports)

    def generate_actions(self):
        """Return a string of newline separated launch actions"""
        launch_actions = []
        for action in self.launch_actions:
            launch_actions.append(action.serialize())
        return ",\n".join(launch_actions)

    def convert_args(self):
        """Find all arg tags in the launch file and convert each to a launch action"""
        for elem in self.xml_tree_root.findall("arg"):
            launch_action = self.convert_arg_elem(elem)
            self.launch_actions.append(launch_action)

    def convert_arg_elem(self, elem):
        """Convert an <arg> element to a ROS2 launch action"""
        default_value = self.convert_attribute(elem.attrib.get('default', None))
        if default_value == None:
            default_value = self.convert_attribute(elem.attrib.get('value', None))
        launch_action = LaunchAction('DeclareLaunchArgument', {
            "name": self.convert_attribute(elem.attrib.get('name', None)),
            "default_value": default_value,
            "description": self.convert_attribute(elem.attrib.get('doc', None))
        })
        return launch_action

    def gather_global_params(self):
        """Convert all <param> elements outside of nodes to global parameters"""
        self.global_params = self.convert_param_elems(self.xml_tree_root)

    def convert_groups(self):
        """Find all <group> elements and convert all sub elements"""
        all_group_nodes = []
        for elem in self.xml_tree_root.findall("group"):
            group_if_attrib = elem.attrib.get('if', None)
            if not group_if_attrib:
                logging.warning("Found group tag without 'if' attribute which is invalid")
                continue
            converted_group_if_attrib = self.convert_attribute(group_if_attrib)
            if_condition = LaunchCondition('IfCondition', converted_group_if_attrib)
            nodes = self.convert_nodes(elem) 
            for node in nodes:
               node.add_condition(if_condition)
            all_group_nodes += nodes
        return all_group_nodes

    def convert_nodes(self, root_elem):
        """Convert all <node> elements to launch actions"""
        nodes = []
        for elem in root_elem.findall("node"):
            launch_action = self.convert_node_elem(elem)
            nodes.append(launch_action)
        return nodes

    def convert_node_elem(self, elem):
        """Convert a single <node> element to a launch action"""
        launch_action = LaunchAction('Node', {
            "package": self.convert_attribute(elem.attrib.get('pkg', None)),
            "node_executable": self.convert_attribute(elem.attrib.get('type', None)),
            "node_name": self.convert_attribute(elem.attrib.get('name', None)),
            "output": self.convert_attribute(elem.attrib.get('output', None))
        })
        required_parameter = elem.attrib.get('required', None)
        if required_parameter and re.match("true", required_parameter, re.IGNORECASE):
            launch_action.set_parameter('on_exit', "launch.actions.Shutdown()")
        node_params = self.global_params + self.convert_param_elems(elem) + self.convert_rosparam_elems(elem)
        if len(node_params) > 0:
            launch_action.set_parameter('parameters', node_params)
        return launch_action

    def convert_attribute(self, value):
        """Convert the value of single attribute 
        
        If the attribute is a string it returns a single quoted string. 
        If the attribute is a string containing an apostraphe it returns a double quoted string
        If the attribute is an argument it converts it to a launch.substitutions.LaunchConfiguration
        which is the ROS2 equivilant of an argument. 
        If the attribute is a find function, convert it to get_package_share_directory 
        which is the ROS2 equivilant of the find function.
        """
        if value == None:
            return None
        arg_search = re.search(r"\$\(arg ([a-zA-Z0-9_]+)\)", value)
        if arg_search != None:
            param_name = arg_search.group(1)
            return f"launch.substitutions.LaunchConfiguration('{param_name}')"
        optenv_search = re.search(r"\$\((?:opt)?env ([a-zA-Z0-9_]+) ?([a-zA-Z0-9_]+)?\)", value)
        if optenv_search != None:
            variable_name = optenv_search.group(1)
            default_value = optenv_search.group(2)
            if default_value == None:
                return f"os.environ.get('{variable_name}')"
            return f"os.environ.get('{variable_name}', '{default_value}')"
        find_search = re.search(r"\$\(find ([a-zA-Z0-9_]+)\)(.*)", value)
        if find_search != None:
            self.imports.update(["from ament_index_python.packages import get_package_share_directory"])
            package = find_search.group(1)
            file_path = find_search.group(2)
            return f"get_package_share_directory('{package}') + '{file_path}'"
        if re.search(r"\'", value):
            return f'"{value}"'
        return f"'{value}'"

    def convert_param_elems(self, node_elem):
        """Convert all <param> tags and add to parameters array""" 
        parameters = []
        param_elems = node_elem.findall("param")
        if len(param_elems) == 0:
            return parameters
        for elem in param_elems:
            param_name = elem.attrib.get('name')
            elem_value = elem.attrib.get('value', None)
            param_value = self.convert_attribute(elem_value)
            parameter = {}
            parameter[param_name] = param_value
            parameters.append(parameter)
        return parameters

    def convert_rosparam_elems(self, node_elem):
        """Convert all <rosparam> tags and add to parameters array""" 
        parameters = []
        param_elems = node_elem.findall("rosparam")
        if len(param_elems) == 0:
            return parameters
        for elem in param_elems:
            param_command = elem.attrib.get('command')
            if param_command != 'load':
                logging.warning("Only 'load' <rosparam> tags are currently supported")
                continue
            param_param = elem.attrib.get('param')
            if param_param is not None:
                logging.warning("<rosparam> tags with a 'param' attribute are not currently supported")
                continue
            param_ns = elem.attrib.get('ns')
            if param_ns is not None:
                logging.warning("<rosparam> tags with a 'ns' attribute are not currently supported")
                continue
            file_path = elem.attrib.get('file', None)
            if file_path is None:
                logging.warning("Only <rosparam> tags with a 'file' parameter are currently supported")
                continue
            param_value = self.convert_attribute(file_path)
            parameters.append(param_value)
        return parameters

    def convert_includes(self):
        """Convert all <include> tags and add to launch_actions array"""
        includes = []
        for elem in self.xml_tree_root.findall("include"):
            launch_action = self.convert_include_elem(elem)
            if launch_action:
                self.imports.update(["from ament_index_python.packages import get_package_share_directory"])
                includes.append(launch_action)
        return includes

    def convert_include_elem(self, elem):
        """Parse a single <include> tag 
        
        This creates and returns an instance of IncludeLaunchDescription which 
        has a serialize() function that is called later to produce the ROS2 equivilant
        of this <include> tag
        """
        file_parameter = elem.attrib.get('file')
        file_info = re.search(r"\$\(find ([a-zA-Z0-9_]+)\)(.*)", file_parameter)
        if file_info == None:
            logging.warning("Found <include> element without 'file' attribute which is invalid")
            return
        package = file_info.group(1)
        launch_file_path = file_info.group(2)
        arguments = {}
        for arg_elem in elem.findall("arg"):
            name = arg_elem.attrib.get('name')
            value = self.convert_attribute(arg_elem.attrib.get('value'))
            if value == None:
                value = self.convert_attribute(arg_elem.attrib.get('default'))
            arguments[name] = value
        launch_action = IncludeLaunchDescription(package, launch_file_path, arguments)
        return launch_action

