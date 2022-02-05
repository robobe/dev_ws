# Hello SKBot

- Create project packages


```bash title="create packages"
mkdir skbot
ros2 pkg create --build-type ament_cmake skbot_description
ros2 pkg create --build-type ament_cmake skbot_gazebo --dependencies gazebo_ros
ros2 pkg create --build-type ament_cmake skbot_control
```

### first build
```
cd dev_ws
colcon build --symlink-install --package-ignore gazebo_pkg
```

## check
```
# check gazebo_ros are installed
ros2 launch gazebo_ros gazebo.launch.py
```

## First world

```xml title="skbot.world"
<?xml version ="1.0"?>
<sdf version="1.4">
    <world name="skbot_world">
        <include>
            <uri>model://sun</uri>
        </include>

        <include>
            <uri>model://ground_plane</uri>
        </include>
    </world>
</sdf>
```
## First launch file

```python
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from  launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_skbot_gazebo = get_package_share_directory("skbot_gazebo")

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

    return LaunchDescription(
        [
            world_arg,
            verbose_arg,
            gazebo
        ]
    )

```

|                               |                                                                      |
| ----------------------------- | -------------------------------------------------------------------- |
| get_package_share_directory   | Return the share directory of the given package (install/share/foo/) |
| IncludeLaunchDescription      |                                                                      |
| DeclareLaunchArgument         |                                                                      |
| PythonLaunchDescriptionSource |                                                                      |


## CMakeLists
- copy `launch` and `world` files to `install` folder

```cmake
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
```

## Build and Run
```bash
colcon build --symlink-install --packages-select skbot_gazebo
ros2 launch skbot_gazebo gz.launch.py
```

## colcon tips
### colcon_cd
cd to package

```
colcon_cd gazebo_ros
```
!!! Note
    Find only packages under current root


!!! Note
    If command not found 
    Source / add to `.bashrc`
    ```
    `source /usr/share/colcon_cd/function/colcon_cd.sh`
    ```



