
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
        
    node=Node(
        name="simple_params",
        package = 'basic',
        executable = 'simple_param',
        parameters = [
            {"my_str": "hello from launch"},
            {"my_int": 1000},
            {"my_double_array": [1.0, 10.0]}
        ]
    )
    ld.add_action(node)
    return ld