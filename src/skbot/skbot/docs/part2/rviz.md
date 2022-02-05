
```
sudo apt install ros-foxy-rviz2
```


## config

!!! Tip "Run RVIZ config from command line"
    ```
    rviz2 -d $(colcon_cd your_package && pwd)/path/to/config.rviz
    ```

### Launch
!!! warning
    Don't forget copy config folder to install folder using CMakelists
    ```
    install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
    ```
```python
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_name = 'skbot_description'
    skbot_description_path = get_package_share_directory(pkg_name)


    rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(skbot_description_path, 'config', 'rviz.rviz')]]
        )

    ld = LaunchDescription()
    ld.add_action(rviz_node)
    return ld
```