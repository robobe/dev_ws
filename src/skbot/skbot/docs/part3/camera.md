

```xml title="urdf_camera sensor" linenums="1" hl_lines="4 32"
<gazebo  reference="camera">
    <material>Gazebo/Blue</material>>

    <sensor type="camera" name="sensor_name">
      <!-- Set always_on only sensor, not on plugin -->
      <always_on>1</always_on>
      <visualize>1</visualize>
      <!-- Set update_rate only sensor, not on plugin -->
      <update_rate>10</update_rate>
      <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>320</width>
          <height>240</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      <camera name="camera_name">
        <distortion>
          <k1>0.0</k1>
          <k2>0.0</k2>
          <k3>0.0</k3>
          <p1>0.0</p1>
          <p2>0.0</p2>
          <!-- <center>0.5 0.5</center> -->
        </distortion>
      </camera>

      <!-- Use camera, not camera_triggered -->
      <plugin name="plugin_name" filename="libgazebo_ros_camera.so">
        <!-- <ros>
          <namespace>custom_ns</namespace>
          <argument>image_raw:=custom_img</argument>
          <argument>camera_info:=custom_info</argument>
        </ros> -->

        <!-- Set camera name. If empty, defaults to sensor name (i.e. "sensor_name") -->
        <!-- <camera_name>custom_camera</camera_name> -->
        <!-- Set TF frame name. If empty, defaults to link name (i.e. "link_name") -->
        <!-- <frame_name>custom_frame</frame_name> -->
        <hack_baseline>0.07</hack_baseline>
      </plugin>
    </sensor>
  </gazebo>
```


![](/images/rviz_add_image_by_topic.png)

![](/images/gazebo_view_image_topic.png)

## image_view

```bash title="install"
sudo apt install ros-fox-image-view
```

### usage
```bash
ros2 topic list
...
/skbot_camera/camera_info
/skbot_camera/image_raw
/skbot/cmd_vel
/skbot/odom
/tf


#
ros2 run image_view image_view image:=/skbot_camera/image_raw
```