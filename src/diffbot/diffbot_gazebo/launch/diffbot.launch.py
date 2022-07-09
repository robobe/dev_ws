import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

pkg_description_pkg = "diffbot_description"
package_name = 'diffbot_gazebo'
xacro_file = "diffbot1.xacro"

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_simulation = get_package_share_directory(package_name)
    pkg_description = get_package_share_directory(pkg_description_pkg)

    # xacro path
    robot_description_path =  os.path.join(
        pkg_description,
        "urdf",
        xacro_file,
    )

    # output urdf path
    urdf_path =  os.path.join(
        pkg_description,
        "urdf",
        xacro_file + ".urdf",
    )

    # convert xacro to urdf
    doc = xacro.process_file(robot_description_path).toxml()
    robot_description = {"robot_description": doc}
    out = xacro.open_output(urdf_path)
    out.write(doc)

    # launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments=[("verbose", "true"), ("pause", "false"), ("world", "diffbot.world")]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", urdf_path, "-entity", "robot", "-x", "0.0", "-y", "0.0", "-z", "0"],
        output="screen")

    # set args before call gazebo
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_robot
    ])