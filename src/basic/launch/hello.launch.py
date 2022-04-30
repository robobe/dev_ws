from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package="basic",
        namespace="my_ns",
        name="my_pub",
        executable="simple_pub",
    )

    listener_node = Node(
        package="basic",
        namespace="my_ns",
        name="my_sub",
        executable="simple_sub"
    )

    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld