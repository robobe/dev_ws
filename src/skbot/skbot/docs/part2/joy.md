# joy

# Joy Teleop
- PS4 joystick (bluetooth)


### pairing
![pair](/images/2022-02-07-17-16-39.png){ width="320" }

### install
```
sudo apt install ros-foxy-joy
```

## todo:
- write node convert from joy msg to twist msg
- [ROS2 USE Joy_node and teleop_twist_joy_node to grab data for moving a robot](https://www.youtube.com/watch?v=suFI5t1_ryg)


```bash
ros2 topic echo /joy
#
header:
  stamp:
    sec: 1644248103
    nanosec: 691350760
  frame_id: joy
axes:
- -0.0
- -0.0
- 1.0
- -0.0
- -0.0
- 1.0
- 0.0
- 0.0
buttons:
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0

```
