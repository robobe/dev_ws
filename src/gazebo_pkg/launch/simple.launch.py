from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

log = logging.get_logger("My_launch")


def generate_launch_description():
    pkg_share = FindPackageShare(package="gazebo_pkg").find("gazebo_pkg")
    gazebo_models_path = os.path.join(pkg_share, "models")
    pkg = get_package_prefix("gazebo_pkg")
    gazebo_plugin_path = os.path.join(pkg, "lib")
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    os.environ["GAZEBO_PLUGIN_PATH"] = gazebo_plugin_path
    world_file_name = "gazebo_ros_template.world"
    world_path = os.path.join(pkg_share, "worlds", world_file_name)

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    ld = LaunchDescription()

    gz_action = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"),
                ),
                launch_arguments={"world": world_path, "verbose": "true"}.items(),
            )
    ld.add_action(gz_action)
    return ld