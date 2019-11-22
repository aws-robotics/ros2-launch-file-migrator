#!/usr/bin/env python3
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

# Contains a class and method for migrating a ROS1 XML launch file to a ROS2 python launch file
import argparse
import logging
import sys

from ros2_launch_file_migrator import LaunchFileMigrator

def main():
    parser = argparse.ArgumentParser(description='Migrate a ROS1 XML Launch file to a ROS2 Python launch file.')
    parser.add_argument('file_path', metavar='file_path', type=str, help='Path to ROS1 launch file. New ROS2 launch file will be saved in the same location with .launch.py extension')
    args = parser.parse_args()
    file_path = args.file_path
    converted_file_path = file_path + ".py"
    file_contents = None
    try:
        with open(file_path, "r") as launch_file:
            file_contents = launch_file.read()
    except FileNotFoundError:
        logging.error(f"Could not find file '{file_path}'.")
        exit(1)
    except PermissionError:
        logging.error(f"Could not open file '{file_path}', check file permissions.")
        exit(1)
    except Exception as e:
        logging.error(e)
        exit(1)

    print(f"Converting file {file_path} to {converted_file_path}")
    ros2LaunchFile = LaunchFileMigrator.migrate(file_contents)
    with open(converted_file_path, "w") as output:
        output.write(ros2LaunchFile)

if __name__ == '__main__':
    main()