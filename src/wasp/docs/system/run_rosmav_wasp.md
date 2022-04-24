# Run wasp


```bash title="terminal1"
ros2 run rosmav vehicle
```

```bash title="terminal2"
ros2 run wasp camera_guide
```

# basic usage
```bash title="terminal3"
ros2 service list
...
/rosmav/arm
/rosmav/change_mode

# call arm
ros2 service call /rosmav/arm rosmav_msgs/srv/CommandBool "{value: True}"

# call change mode
ros2 service call /rosmav/change_mode rosmav_msgs/srv/CommandInt "{value: 20}"
```

