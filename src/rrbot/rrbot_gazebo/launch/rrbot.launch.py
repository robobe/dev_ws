import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

pkg_description_pkg = "rrbot_description"
package_name = 'rrbot_gazebo'
xacro_file = "rrbot.xacro"

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
        )
    )

    verbose_arg = DeclareLaunchArgument(
            'verbose', default_value='true',
            description='Set "true" to increase messages written to terminal.'
        )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
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
        arguments=["-file", urdf_path, "-entity", "robot", "-x", "0.0", "-y", "0.0", "-z", "0.0"],
        output="screen")

    # set args before call gazebo
    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        verbose_arg,
        gazebo,
        spawn_robot
    ])