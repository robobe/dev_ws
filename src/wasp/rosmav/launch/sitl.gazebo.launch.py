import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    shared_location = get_package_share_directory('rosmav')
    copter_parm = os.path.join(shared_location, "config", "copter.parm")
    gazebo_parm = os.path.join(shared_location, "config", "gazebo-iris.parm")
    # ~/apm/copter410/arducopter --model + --defaults ~/apm/copter410/copter.parm -I0
    sitl_process = ExecuteProcess(cmd=[
            "/home/user/apm/copter410/arducopter",
            "--model",
            "gazebo-iris",
            "--defaults",
            f"{copter_parm},{gazebo_parm}",
            "-I0"
        ],
        cwd="/home/user/apm/copter410",
        name="SITL",
        output="both")

    
    ld.add_action(sitl_process)

    return ld