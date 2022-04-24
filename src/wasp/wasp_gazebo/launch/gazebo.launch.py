import os
from launch.actions import SetEnvironmentVariable
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from  launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    wasp_gazebo = get_package_share_directory("wasp_gazebo")


    gz_model_path = os.environ.get("GAZEBO_RESOURCE_PATH")

    gz_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH", 
        value=[os.path.join(wasp_gazebo + '/models'),
        ":" + gz_model_path
        ]
    )

    gz_plugins_path = os.environ.get("GAZEBO_RESOURCE_PATH")

    gz_plugins_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH", 
        value=['/home/user/dev_ws/install/wasp_gazebo/',
        ":" + gz_plugins_path
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"))
    )

    world_arg = DeclareLaunchArgument("world",
            default_value=[os.path.join(wasp_gazebo, "worlds", "iris_arducopter_runway.world"), ""],
            description="hello wasp world"
            )

    verbose_arg = DeclareLaunchArgument("verbose",
            default_value=["true"],
            description="verbose log"
            )

    return LaunchDescription(
        [
            gz_plugins_path,
            gz_model_path,
            world_arg,
            verbose_arg,
            gazebo
        ]
    )