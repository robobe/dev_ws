---
title: service interface
tags:
    - service
    - interface
---

!!! warning "interface"
     ROS2 interfaces `msg` and `srv` must be `ament_cmake` package


```bash
ros2 pkg create skbot_interfaces --build-type ament_cmake
```

## pkg 
- Add msg folder
- Add srv folder
- Add support in CMakelists.txt
- Add idl support in package.xml

### pkg folders and files
```linenums="1" hl_lines="4 5"
├── CMakeLists.txt
├── msg
├── package.xml
├── skbot_interfaces
│   └── __init__.py
└── srv
    └── AddTwoInts.srv

```

### srv
- Add service message files (`.srv`) under `srv` folder
  - for example `AddTwoInts.srv`

```title="AddTwoInts.srv"
int64 a
int64 b
---
int64 sum
```

### CMakeLists.txt
- Add rosidl support
- Add python support (module discovery)
  
!!! warning "python module discovery"
    Support python module discovery
    - Create folder with package name  
    - Add `__init__.py` file under this folder  
    - Add `ament_python_install_package` command to CMakeLists  

    ```cmake
    ament_python_install_package(${PROJECT_NAME})
    ```
     

```cmake title="CMakeLists.txt addition"
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
)

# Python model discovery
ament_python_install_package(${PROJECT_NAME})
```

### package.xml
- Add rosidl support

```xml title="package.xml addition"
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

```

### Build and check
- colcon
- source `install`
- check

```bash
colcon build --symlink-install --packages-select skbot_interfaces 

source install/setup.bash 

# list all interfaces
ros2 interface list

# show interface
ros2 interface show skbot_interfaces/srv/AddTwoInts 
```

