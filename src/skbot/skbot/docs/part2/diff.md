# Diff Drive
- urdf
- launch
- rviz
- gazebo
- teleop
- Pub Twist message
---
## URDF
- Add gazebo_ros_joint_state_publisher plugin
- Add gazebo_ros_diff_drive plugin

```xml title="skbot.gazebo" linenums="1" hl_lines="2 9"
<gazebo>
  <plugin name="joint_states" filename="libgazebo_ros_joint_state_publisher.so">
    <joint_name>right_wheel_hinge</joint_name>
    <joint_name>left_wheel_hinge</joint_name>
  </plugin>
</gazebo>

<gazebo>
     <plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive_controller">

          <ros>
          <namespace>/skbot</namespace>
          </ros>

          <!-- wheels -->
          <left_joint>left_wheel_hinge</left_joint>
          <right_joint>right_wheel_hinge</right_joint>

          <!-- kinematics -->
          <wheel_separation>${chassisWidth+wheelWidth}</wheel_separation>
          <wheel_diameter>${2*wheelRadius}</wheel_diameter>

          <!-- limits -->
          <max_wheel_torque>10</max_wheel_torque>
          <max_wheel_acceleration>1.0</max_wheel_acceleration>

          <!-- output -->
          <publish_odom>true</publish_odom>
          <publish_odom_tf>true</publish_odom_tf>

          <odometry_frame>odom</odometry_frame>
          <robot_base_frame>chassis</robot_base_frame>
     </plugin>
</gazebo>
```

```xml title="ros section inside plugin tag"
<ros>
  <!-- Set namespace -->
  <namespace>/demo</namespace>

  <!-- Remap default topics -->
  <argument>cmd_vel:=cmd_demo</argument>
  <argument>odom:=odom_demo</argument>
</ros>
```
!!! warning "joint_publisher"
     remove joint_publisher from launch file

---

# Launch
```python title="gz_rviz.launch.py"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    skbot_description = get_package_share_directory("skbot_description")

    robot_description_path =  os.path.join(
        skbot_description,
        "urdf",
        "skbot.xacro",
    )
    urdf_path =  os.path.join(
        skbot_description,
        "urdf",
        "skbot.urdf")
    
    doc = xacro.process_file(robot_description_path).toxml()
    out = xacro.open_output(urdf_path)
    out.write(doc)
  
    return LaunchDescription([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                arguments=[urdf_path]),
            Node(
                package='rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', [os.path.join(skbot_description, 'config', 'map.rviz')]]),
            ExecuteProcess(
                cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
                output='screen'),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='urdf_spawner',
                output='screen',
                arguments=["-topic", "/robot_description", "-entity", "skbot"])
    ])
```
    
--- 

## Rviz
- Running rviz using map.rviz config
     - Set `Fixed Frame` to odom


![](/images/rviz_map.png)

---

## Teleop

``` title="install"
sudo apt install ros-foxy-teleop-twist-keyboard
```

```title="run and mapping"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/skbot/cmd_vel
```

--- 

!!! note "Terminator"
     ```
     sudo apt install terminator
     ```

     ```
     Ctrl+Shift+E 	vertical split
     Ctrl+Shift+O 	horizontal split
     Alt+ArrowKeys 	Navigate terminals
     ```     

---

## Pub Twist message
```
ros2 topic pub -1 /skbot/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0" 

```

