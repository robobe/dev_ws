
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
        
    my_str_arg = DeclareLaunchArgument("my_str", default_value="world")
    my_str = LaunchConfiguration("my_str")    
    node=Node(
        name="simple_params",
        package = 'basic',
        executable = 'simple_param',
        parameters = [
            {"my_str": my_str},
            {"my_int": 1000},
            {"my_double_array": [1.0, 10.0]}
        ]
    )
    ld.add_action(my_str_arg)
    ld.add_action(node)
    return ld