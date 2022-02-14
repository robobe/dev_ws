---
title: xacro args
tags:
    - xacro
    - args
    - launch
---
Using launch to pass argument to xacro file
- Declare `xacro:arg` 
- Use it for example in `xacro:if`
- Add `mappings` to `xacro.process_file`

```xml title="test.sdf.xacro" linenums="1" hl_lines="4 13"
<?xml version="1.0"?>
<sdf version="1.7"
    xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:arg name="sphere" default="true" />

    <xacro:include filename="/home/user/dev_ws/vscode/sdf/templates/macros.xacro" />
    <xacro:include filename="/home/user/dev_ws/vscode/sdf/templates/common.xacro" />
    
    <model name="test">
        <link name="link_00">
            <visual name="visual_00">
                <geometry>
                    <xacro:if value="$(arg sphere)">
                        <sphere>
                            <radius>${PI}</radius>
                        </sphere>
                    </xacro:if>
                </geometry>
            </visual>
        </link>
    </model>
</sdf>
```
```python title="xacro_args.launch.py" linenums="1" hl_lines="21"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import xacro

def generate_launch_description():
    pkg_skbot_gazebo = get_package_share_directory("skbot_gazebo")

    model_path =  os.path.join(
        pkg_skbot_gazebo,
        "models",
        "test",
        "test.sdf.xacro"
    )
    urdf_path =  os.path.join(
        pkg_skbot_gazebo,
        "models",
        "test",
        "test.sdf")

    doc = xacro.process_file(model_path, mappings={"sphere": "true"}).toxml()
    out = xacro.open_output(urdf_path)
    out.write(doc)

    return LaunchDescription([])
```