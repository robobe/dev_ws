

# colcon

## install
```
sudo apt install python3-colcon-common-extensions
```

```bash
source /usr/share/gazebo/setup.bash
source /opt/ros/foxy/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/dev_ws
```

## cheat-sheet
```
colcon build
colcon build --packages-select <name-of-pkg>
colcon build --symlink-install <name-of-pkg>
```


## colcon_cd
