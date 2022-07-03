import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

package_name = 'rrbot_description'
urdf_name = "rrbot.urdf"

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "config.rviz")

    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        urdf_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
  
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
  
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )
    
    return launch.LaunchDescription([
        use_sim_time_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
])