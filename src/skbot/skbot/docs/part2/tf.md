# TF

## robot_state_publisher
uses the URDF specified by the parameter `robot_description` and the joint positions from the topic `joint_states` to calculate the forward kinematics of the robot and publish the results via `tf`.

```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro src/skbot/skbot_description/urdf/skbot.xacro)"
```

## joint_state_publisher
This package publishes sensor_msgs/JointState messages for a robot. The package reads the robot_description parameter, finds all of the non-fixed joints and publishes a JointState message with all those joints defined.


![](/images/state_publisher.drawio.svg)

## launch urdf with robot_description
```python title="robot_state_publisher" linenums="1" hl_lines="20"
#!/usr/bin/env python3
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_skbot_description = get_package_share_directory("skbot_description")
    robot_description_path =  os.path.join(
        pkg_skbot_description,
        "urdf",
        "skbot.xacro",
    )

    doc = xacro.parse(open(robot_description_path))
    xacro.process_doc(doc)

    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[params])

    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_joint_state_publisher)
    return ld

```
---

## TF Tree
- using rqt

```
sudo apt install ros-foxy-rqt-tf-tree
```

### usage

```bash
ros2 run rqt_gui rqt_gui
#
# select Plugins -> Visualization -> TF Tree
```
![](/images/rqt_tf_tree.png)

# References
- [Getting Ready for ROS Part 6: The Transform System (TF)](https://articulatedrobotics.xyz/ready-for-ros-6-tf/)