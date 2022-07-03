import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

package_name = 'rrbot_description'
urdf_name = "rrbot.urdf"

def generate_launch_description():
  rviz_config_path = os.path.join(
    get_package_share_directory(package_name),
    "config",
    "config.rviz")

  urdf_path = os.path.join(
    get_package_share_directory(package_name),
    "urdf",
    urdf_name)

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    arguments=[urdf_path]
  )

  joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher'
  )

  joint_state_publisher_gui_node = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui'
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_path],
  )

  return LaunchDescription([
    joint_state_publisher_node,
    joint_state_publisher_gui_node,
    robot_state_publisher_node,
    rviz_node
  ])