---
title: read scan/laser data
tags:
    - hello
    - laser
    - scan
---

```bash title="topic list"
ros2 topic list
#
/joint_states
/robot_description
/rosout
/scan
/skbot/cmd_vel
/skbot/odom
/skbot_camera/camera_info
/skbot_camera/image_raw
```

```bash title="topic info"
ros2 topic info /scan
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Subscription count: 0
```

```bash
ros2 interface show sensor_msgs/msg/LaserScan
#
std_msgs/Header header  
float32 angle_min       
float32 angle_max       
float32 angle_increment 
float32 time_increment  
float32 scan_time       
float32 range_min       
float32 range_max       
float32[] ranges        
float32[] intensities   
```


```bash title="info verbose" hl_lines="12"
ros2 topic info --verbose /scan
Type: sensor_msgs/msg/LaserScan

Publisher count: 1

Node name: laser
Node namespace: /
Topic type: sensor_msgs/msg/LaserScan
Endpoint type: PUBLISHER
GID: 01.0f.d1.c7.1b.a2.89.9c.01.00.00.00.00.00.4f.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 0

```

## Simple simple subscriber
```python title="hello_laser.py" linenums="1" hl_lines="3 4 9 14"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MyNode(Node):
    def __init__(self):
        super().__init__('skbot_motion_scan')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
            depth=1
        )
        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.__scan_handler,
            qos_profile
        )

    def __scan_handler(self, msg: LaserScan):
        self.get_logger().info(msg.header.frame_id)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
``` 

