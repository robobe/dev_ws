---
title: hello python node
tags:
    - hello
    - python
---

## Node (Hello)

```python title="hello.py" 
import rclpy
from rclpy.node import Node
class MyNode(Node):
    def __init__(self):
        super().__init__('skbot_motion')
        self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello skbot")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
```

## setup.py
- Add entry node

```python hl_lines="3"
entry_points={
        'console_scripts': [
            'hello = skbot_motion.hello:main'
        ],
    }
```

## package.xml

```xml
<exec_depend>rclpy</exec_depend>
```

## build and run
```bash
# Build
colcon build --symlink-install --packages-select skbot_motion 
# Source
source install/setup.bash
# Run
ros2 run skbot_motion hello
```

# References
- [m2wr](https://github.com/mattborghi/m2wr_description)
- [Write a Minimal ROS2 Python Node](https://roboticsbackend.com/write-minimal-ros2-python-node/)