import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

pkg_description_pkg = "rrbot_description"
package_name = 'rrbot_gazebo'

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_description = get_package_share_directory(pkg_description_pkg)

    urdf_path =  os.path.join(
        pkg_description,
        "urdf",
        "box.urdf",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments=[("verbose", "true")]
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", urdf_path, "-entity", "robot", "-x", "0.0", "-y", "0.0", "-z", "0.0"],
        output="screen")

    return LaunchDescription([
        gazebo,
        spawn_robot
    ])