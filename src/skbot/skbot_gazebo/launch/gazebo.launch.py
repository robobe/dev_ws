from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

package_name = "skbot_gazebo"
world_file = "empty.world"


def generate_launch_description():

    ld = LaunchDescription()

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_simulation = get_package_share_directory(package_name)

    # launch Gazebo by including its definition
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"),
        )
    )

    # load the world file
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=[os.path.join(pkg_simulation, "worlds", world_file), ""],
        description="SDF world file",
    )

    ld.add_action(gazebo)
    ld.add_action(world_arg)
    return ld
