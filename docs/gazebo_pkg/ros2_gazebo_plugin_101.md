---
title: ROS2 gazebo plugin
tags:
    - plugin
    - template
    - hello 101
---

Create first ROS2 gazebo plugin following `gazebo_ros_template` from [gazebo_ros_pkgs github](https://github.com/ros-simulation/gazebo_ros_pkgs)

```
gazebo_pkg/
├── CMakeLists.txt
├── include
│   └── gazebo_pkg
│       └── gazebo_ros_template.hpp
├── launch
│   └── simple.launch.py
├── package.xml
├── src
│   └── gazebo_ros_template.cpp
└── worlds
    └── gazebo_ros_template_demo.world
```

## Plugin
<details>
    <summary>header file</summary>
    
```cpp title="gazebo_ros_template.hpp" linenums="1" hl_lines="10 12"
{{include("src/gazebo_pkg/include/gazebo_pkg/gazebo_ros_template.hpp")}}
```
</details>

<details>
    <summary>plugin file</summary>
    
```cpp title="gazebo_ros_template.cpp" linenums="1" hl_lines="14 18 28"
{{include("src/gazebo_pkg/src/gazebo_ros_template.cpp")}}
```
</details>

<details>
    <summary>cmake file</summary>

```cmake title="CMakeLists.txt" linenums="1" 
{{include(("src/gazebo_pkg/CMakeLists.txt"))}}
```
</details>

## World file
<details>
    <summary>world file</summary>

```xml title="gazebo_ros_template.world" 
{{include("src/gazebo_pkg/worlds/gazebo_ros_template.world")}}
```
</details>

## Launch
<details>
    <summary>launch file</summary>

```python title="simple.launch.py"
{{include("src/gazebo_pkg/launch/simple.launch.py")}}
```
</details>

### Run
```
ros2 launch gazebo_pkg simple.launch.py
```

![](/images/2022-01-29-07-41-45.png)