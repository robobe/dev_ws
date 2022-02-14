import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import xacro

def generate_launch_description():
    pkg_skbot_gazebo = get_package_share_directory("skbot_gazebo")

    model_path =  os.path.join(
        pkg_skbot_gazebo,
        "models",
        "test",
        "test.sdf.xacro"
    )
    urdf_path =  os.path.join(
        pkg_skbot_gazebo,
        "models",
        "test",
        "test.sdf")

    doc = xacro.process_file(model_path, mappings={"sphere": "true"}).toxml()
    out = xacro.open_output(urdf_path)
    out.write(doc)

    return LaunchDescription([])