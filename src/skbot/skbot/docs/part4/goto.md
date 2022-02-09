---
title: go to 
tags:
    - diff
    - goto
---
Command skbot goto point  
- Read odom topic  
- Send Twist msg

!!! tip "echo message"
    echo part of message using grep
    ```
    ros2 topic echo /skbot/odom | grep -A 3 "position"
    ros2 topic echo /skbot/odom | grep -A 4 "orientation"
    ```
     
## Code
```python title="goto.py" linenums="1" hl_lines="63 128"
{{include("skbot_motion/skbot_motion/goto_point.py")}}
```

!!! note ""
    Transforms3d
    ```
    pip install transforms3d
    ```

!!! warning "euler and quaterninon"
    Try to use transforms3d `quat2euler` without success  
    move to util function

    ```python
    def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z
    ```
     
     