
# Launch
- Using `gazebo_ros` package to launch gazebo
- Declare world and gazebo command args

```python title="gz_v2.launch.py" linenums="1" hl_lines="29 33 38"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from  launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_skbot_description = get_package_share_directory("skbot_description")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_skbot_gazebo = get_package_share_directory("skbot_gazebo")

    robot_description_path =  os.path.join(
        pkg_skbot_description,
        "urdf",
        "skbot.xacro",
    )
    urdf_path =  os.path.join(
        pkg_skbot_description,
        "urdf",
        "skbot.urdf")
    
    doc = xacro.process_file(robot_description_path).toxml()
    out = xacro.open_output(urdf_path)
    out.write(doc)
  
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

    robot_state_publisher_node = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                arguments=[urdf_path])
            
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
        gazebo,
        spawn_robot_node
    ])
``` 