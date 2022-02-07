import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    skbot_description = get_package_share_directory("skbot_description")

    robot_description_path =  os.path.join(
        skbot_description,
        "urdf",
        "skbot.xacro",
    )
    urdf_path =  os.path.join(
        skbot_description,
        "urdf",
        "skbot.urdf")
    
    doc = xacro.process_file(robot_description_path).toxml()
    out = xacro.open_output(urdf_path)
    out.write(doc)
  
    return LaunchDescription([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                arguments=[urdf_path]),
            Node(
                package='rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', [os.path.join(skbot_description, 'config', 'map.rviz')]]),
            ExecuteProcess(
                cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
                output='screen'),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='urdf_spawner',
                output='screen',
                arguments=["-topic", "/robot_description", "-entity", "skbot"])
    ])