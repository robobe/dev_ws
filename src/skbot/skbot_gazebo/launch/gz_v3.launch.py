import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.logging import logging 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

log = logging.getLogger()

GAZEBO_WORLD = "world01.world"

def generate_launch_description():
    pkg_skbot_description = get_package_share_directory("skbot_description")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_skbot_gazebo = get_package_share_directory("skbot_gazebo")
    world_path = os.path.join(pkg_skbot_gazebo, "worlds", GAZEBO_WORLD)

    robot_description_path =  os.path.join(
        pkg_skbot_description,
        "urdf",
        "skbot.xacro",
    )
    robot_description_meshes =  os.path.join(
        pkg_skbot_description,
        "meshes"
    )

    gazebo_resource_path = os.environ.get("GAZEBO_RESOURCE_PATH")

    gazebo_resource_path = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", 
        value=[robot_description_meshes,
        ":" + gazebo_resource_path
        ]
    )
    robot_description_raw = xacro.process_file(robot_description_path).toxml()
      
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"))
    )

    world_arg = DeclareLaunchArgument("world",
            default_value=[world_path, ""],
            description="hello skbot world"
            )

    verbose_arg = DeclareLaunchArgument("verbose",
            default_value=["true"],
            description="verbose log"
            )

    robot_state_publisher_node = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description_raw,
                    'use_sim_time': True
                }])
            
    spawn_robot_node = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='urdf_spawner',
                output='screen',
                arguments=["-topic", "/robot_description", "-entity", "skbot"])

    return LaunchDescription([
        robot_state_publisher_node,
        world_arg,
        verbose_arg,
        gazebo_resource_path,
        gazebo,
        spawn_robot_node
    ])