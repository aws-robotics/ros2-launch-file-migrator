from setuptools import setup, find_packages

package_name = "ros2_launch_file_migrator"

setup(
  name=package_name,
  version="0.3",
  description="Migrates a ROS1 XML launch file into a ROS2 Python launch file",
  url="",
  author="AWS RoboMaker",
  author_email="ros-contributions@amazon.com",
  license="MIT",
  packages=find_packages(),
  package_data={
    "ros2_launch_file_migrator": ["templates/*"]
  },
  scripts=[package_name + "/bin/migrate_launch_file"],
  zip_save=True
)
