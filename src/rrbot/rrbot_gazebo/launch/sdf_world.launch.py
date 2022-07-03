import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

pkg_description_pkg = "rrbot_description"
package_name = 'rrbot_gazebo'
world_file = 'sdf.world'

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_simulation = get_package_share_directory(package_name)
    

    # launch Gazebo by including its definition
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments=[("verbose", "true")]
    )


    # load the world file
    world_arg = DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_simulation, 'worlds', world_file), ''],
          description='SDF world file')

    return LaunchDescription([
        world_arg,
        gazebo
    ])