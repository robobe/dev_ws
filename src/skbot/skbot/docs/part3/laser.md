---
title: laser scan
tags:
    - ray
    - gpy ray
---

## urdf
### urdf with mesh
- mesh file location `skbot_description/meshes` folder
- Add this folder to gazebo env. variable `GAZEBO_RESOURCE_PATH` (launch file)

!!! note "load resource"
  - file
  - model

  ```xml
  <!-- file -->
  <visual name='base_link_visual'>
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <mesh>
        <scale>1 1 1</scale>
        <uri>file://path_to_dae/textured.dae</uri>
      </mesh>
    </geometry>
  </visual>

  <!-- model -->
  <visual name='base_link_visual'>
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <mesh>
        <scale>1 1 1</scale>
        <uri>model://model_name/meshes/textured.dae</uri>
      </mesh>
    </geometry>
  </visual>
  ```
     
```xml title="laser sensor" linenums="1" hl_lines="12"
<link name="hokuyo">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="file://hokuyo.dae"/>
      </geometry>
    </visual>

    <inertial>
      <mass value="1e-5" />
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>
```
## gazebo

```xml title="laser sensor" linenums="1" hl_lines="2 28"
<gazebo reference="hokuyo">
    <sensor name="sensor_ray" type="ray">
      <pose>0.0 0 0 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1.0</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.120000</min>
          <max>3.5</max>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <always_on>true</always_on>
      <visualize>true</visualize>
      <topic>scan</topic>

      <update_rate>40.0</update_rate>
      <plugin name="laser" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <!-- <frame_name>base_scan</frame_name> -->
      </plugin>
    </sensor>
  </gazebo>
```

!!! tip "check for ray_gpu"
     Check ROS2 ray_gpu sensor

---

## launch
- Add gazebo environment variables (GAZEBO_RESOURCE_PATH)

<details>
    <summary>gazebo launch file</summary>

```python title="gz_v3.launch.py" linenums="1" hl_lines="30 32 74"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.logging import logging 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

log = logging.getLogger()

def generate_launch_description():
    pkg_skbot_description = get_package_share_directory("skbot_description")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_skbot_gazebo = get_package_share_directory("skbot_gazebo")

    robot_description_path =  os.path.join(
        pkg_skbot_description,
        "urdf",
        "skbot.xacro",
    )
    robot_description_meshes =  os.path.join(
        pkg_skbot_description,
        "meshes"
    )

    gazebo_resource_path = os.environ.get("GAZEBO_RESOURCE_PATH")

    gazebo_resource_path = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", 
        value=[robot_description_meshes,
        ":" + gazebo_resource_path
        ]
    )
    robot_description_raw = xacro.process_file(robot_description_path).toxml()
      
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
                parameters=[{
                    'robot_description': robot_description_raw,
                    'use_sim_time': True
                }])
            
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
        gazebo_resource_path,
        gazebo,
        spawn_robot_node
    ])
```
</details>

--- 

## Rviz
- Add `LaserScan` with topic `scan`
- Change `LaserScan/Size(m)` to `0.05` for better visualization


![](/images/rviz_laser_scan.png)