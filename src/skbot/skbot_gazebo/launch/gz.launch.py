import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from  launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_skbot_gazebo = get_package_share_directory("skbot_gazebo")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"))
    )

    world_arg = DeclareLaunchArgument("world",
            default_value=[os.path.join(pkg_skbot_gazebo, "worlds", "skbot.world"), ""],
            description="hello skbot world"
            )

    verbose_arg = DeclareLaunchArgument("verbose",
            default_value=["true"],
            description="verbose log"
            )

    return LaunchDescription(
        [
            world_arg,
            verbose_arg,
            gazebo
        ]
    )
