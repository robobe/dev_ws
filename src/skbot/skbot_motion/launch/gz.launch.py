import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from  launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_skbot_gazebo = get_package_share_directory("skbot_gazebo")
    pkg_skbot_description = get_package_share_directory("skbot_description")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"))
    )

    world_arg = DeclareLaunchArgument("world",
            default_value=[os.path.join(pkg_skbot_gazebo, "worlds", "world02.world"), ""],
            description="hello skbot world"
            )

    verbose_arg = DeclareLaunchArgument("verbose",
            default_value=["true"],
            description="verbose log"
            )

    # DeclareLaunchArgument('x', default_value='-5.0'),
    # DeclareLaunchArgument('y', default_value='-5.0'),
    # DeclareLaunchArgument('z', default_value='0.0'),
    # DeclareLaunchArgument('R', default_value='0.0'),
    # DeclareLaunchArgument('P', default_value='0.0'),
    # DeclareLaunchArgument('Y', default_value='0.0'),
    mesh_path = os.path.join(pkg_skbot_description, "meshes")
    gazebo_resource_path = SetEnvironmentVariable('GAZEBO_RESOURCE_PATH',
        mesh_path + ':' + os.environ["GAZEBO_RESOURCE_PATH"])

    robot_description_path =  os.path.join(
        pkg_skbot_description,
        "urdf",
        "skbot.xacro",
    )

    urdf_path =  os.path.join(
        pkg_skbot_description,
        "urdf",
        "skbot.urdf",
    )

    doc = xacro.process_file(robot_description_path).toxml()
    out = xacro.open_output(urdf_path)
    out.write(doc)

    # gz model --spawn-file=/home/user/dev_ws/install/skbot_description/share/skbot_description/urdf/skbot.urdf --model-name=skbot
    spawn = ExecuteProcess(
            cmd=[
                'gz', 'model',
                '--spawn-file', urdf_path,
                '--model-name', 'skbot',
                '-x', "5",
                '-y', "-5"
                
            ],
            prefix="bash -c 'sleep 5s; $0 $@'",
            output='screen')

    return LaunchDescription(
        [
            world_arg,
            verbose_arg,
            gazebo_resource_path,
            gazebo,
            spawn
        ]
    )
