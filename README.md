# ROS2 Launch File Migrator

This package takes a ROS1 XML launch file and converts it to a ROS2 Python launch file

## Requirements

- Python 3.6+

## Installation

```
pip3 install -e ./ --user
```

## Usage

As a standalone script:
```
migrate_launch_file path/to/launch_file.launch
```

In another script
```
from ros2_launch_file_migrator import LaunchFileMigrator
converted_code =  LaunchFileMigrator.migrate(existing_launch_file_code)
```
## Test

```
python3 -m pytest
```

## Supported Tags

- `<node>`
- `<arg>`
- `<param>`
- `<include>`
- `<group>`
- `<rosparam>` - Basic config file loading is supported. Custom namespaces are not supported.

## Unsupported Tags

You can still convert a launch file with unsupported tags, they will just be skipped. 

- `<remap>`

## Known Issues

- In ROS2 you there is a bug where you cannot set the `output` parameter of a node via an argument 
(see https://github.com/ros2/launch_ros/issues/46). This is possible in ROS1 so this 
launch file converter will faithfully convert your launch file correctly but it won't 
work until the bug is fixed. 


## License

This library is licensed under the Apache 2.0 License.

**Author**: AWS RoboMaker
**Affiliation**: [Amazon Web Services (AWS)](https://aws.amazon.com)
**Maintainer**: AWS RoboMaker, ros-contributions@amazon.com
