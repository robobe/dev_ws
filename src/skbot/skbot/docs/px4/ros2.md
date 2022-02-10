---
title: PX4 ROS2
tags:
    - px4
    - ros2
    - fast dds
---

[ROS 2 User Guide (PX4-ROS 2 Bridge)](https://docs.px4.io/master/en/ros/ros2_comm.html#build-ros-2-workspace)

![](/images/px_ros2_bridge.png)

## Setup
- Install Fast DDS
- Build ROS2 workspace

### Fast DDS
[Fast DDS Installation](https://docs.px4.io/master/en/dev_setup/fast-dds-installation.html)

!!! warning "Gradle"
    Recommend version 6.3
    - Download binary from [... todo]()
    - Unzip to `/opt` folder (or other)
    - Fix PATH

!!! tip "install"
    Recommend to compile from source
    - Ubuntu 20.04
      - Foonathan 
      - Fast DDS 2.0.2
      - Fast RTSP-Gen 1.0.4 (not later or grater)
      - java JDK install by PX4 ubuntu setup script

!!! tip "FASTRTPSGEN_DIR"
    Set environment variable if not a default installation

---

# ROS2 Workspace
[Build ROS 2 Workspace](https://docs.px4.io/master/en/ros/ros2_comm.html#build-ros-2-workspace)

```bash title="Terminal 1"
# SITL start micrortps_client on port 2019,2020
make px4_sitl_rtps gazebo
```

```bash title="Terminal2"
micrortps_agent -t UDP
```


```bash title="Terminal3"
# my demo node
# hello -> run sensor_combain
ros2 run my_px4 hello 
```

## ROS2 Example

```python
import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MyNode(Node):
    def __init__(self):
        super().__init__('my_px4')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
            depth=1
        )
        self.create_subscription(SensorCombined,
            "fmu/sensor_combined/out",
            self.__handler,
            qos_profile)

    def __handler(self, msg):
        self.get_logger().info(str(msg.gyro_rad[0]))

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

# References
- [Fast DDS Installation](https://docs.px4.io/master/en/dev_setup/fast-dds-installation.html)
- [ROS 2 User Guide (PX4-ROS 2 Bridge)](https://docs.px4.io/master/en/ros/ros2_comm.html#build-ros-2-workspace)
- [ROS 2 Offboard Control Example](https://docs.px4.io/master/en/ros/ros2_offboard_control.html)
     
     
     