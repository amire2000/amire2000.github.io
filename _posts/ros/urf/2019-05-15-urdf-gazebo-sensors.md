---
layout: post
title: URDF gazebo sensor integration
categories: ros
tags: [urdf, gazebo, sensors]
description: Embedded gazebo sensors and other behavior in urdf file, View camera, depth camera and other sensors in gazebo and rviz
image: urdf_sdf.jpg
public: true
---
*URDF* (Universal Robotics Description Format) is an XML file format used in ROS to describe all element of a robot.  
*SDF* (Simulation Description Format) used by gazebo to describe everything from the world level down to the robot level.  

URDF can only specify the kinematic and dynamic properties of a single robot.  

To use a URDF file in gazebo , `<gazebo>` element introduce
- Add a `<gazebo>` element for `<link>`
  - convert visual color to gazebo format
  - Add sensor and plugins
- Add a `<link name="world"/>` link if the robot should  rigidly attached to world 
  
# Content
- [Camera](#add-camera-to-urdf-file)
- [Depth camera](#)
- 
# Add camera to urdf file
- Add camera sensor
- Add ROS plugin
- Use xacro for reuse code
  - Declare link xacro with visual collision and inertial parts
- Wrap sensor and plugin with `gazebo` element

```xml
<gazebo reference="sensor">
    <sensor type="camera" name="camera1">
        <update_rate>30.0</update_rate>
        <visualize>1</visualize>
        <pose frame="sensor">${width/2} 0 ${height1/2} 0 0 0</pose>
        <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
            <width>800</width>
            <height>800</height>
            <format>R8G8B8</format>
        </image>
        <clip>
            <near>0.02</near>
            <far>300</far>
        </clip>
        <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
        </noise>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>0.0</updateRate>
            <cameraName>/mysensor/camera</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>camera_link</frameName>
            <hackBaseline>0.07</hackBaseline>
            <distortionK1>0.0</distortionK1>
            <distortionK2>0.0</distortionK2>
            <distortionK3>0.0</distortionK3>
            <distortionT1>0.0</distortionT1>
            <distortionT2>0.0</distortionT2>
        </plugin>
    </sensor>
</gazebo>
```
## Code explain
### `gazebo` sensor tag  

gazebo tag wrap sensor sdf declaration
the `gazebo` tag point to URDf element in the file.

`<gazebo reference="sensor"> ... </gazebo>`

### `gazebo` color tag
TBD

# Launch , view in gazebo rviz and topics

![](/images/2019-05-16-07-31-52.png)

## camera topics
```bash
rostopic list | grep camera

/mysensor/camera/camera_info
/mysensor/camera/image_raw
/mysensor/camera/image_raw/compressed
/mysensor/camera/image_raw/compressed/parameter_descriptions
/mysensor/camera/image_raw/compressed/parameter_updates
/mysensor/camera/image_raw/compressedDepth
/mysensor/camera/image_raw/compressedDepth/parameter_descriptions
/mysensor/camera/image_raw/compressedDepth/parameter_updates
/mysensor/camera/image_raw/theora
/mysensor/camera/image_raw/theora/parameter_descriptions
/mysensor/camera/image_raw/theora/parameter_updates
/mysensor/camera/parameter_descriptions
/mysensor/camera/parameter_updates
```

### camera_info
TBD

### image_raw, image_raw/compressed, image_raw/theora
TBD

## View in rviz
- View images from camera image_raw topics
  - raw
  - compressed
  - theora
  
![](/images/2019-05-16-07-29-35.png)

# Package 
The package / demo project base on `ros_robotics_projects/chapter_10_codes/sensor_sim_gazebo/` see references

Its include launch file and sensor xacro/urdf file for each sensor
All sensors include base link file with visual and inetia data

## camera launch 
```xml
<?xml version="1.0" ?>
<launch>
  <arg name="verbose" default="true"/>
  <arg name="world_name" default="worlds/empty.world"/> 
  <include file="$(find gazebo_ros)/launch/empty_world.launch" >
    <arg name="verbose" value="$(arg verbose)"/>
    <arg name="world_name" value="$(arg world_name)"/>
  </include>

  <!-- Spawn the example robot -->
  <param name="robot_description" command="$(find xacro)/xacro '$(find gazebo_integration)/urdf/camera.xacro'" />

  <node pkg="gazebo_ros" type="spawn_model" name="spawn_model" args="-urdf -param /robot_description -model example"/>

  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher"/>

</launch>
```
&nbsp;  
&nbsp;  
&nbsp;  
# Depth
Depth camera based on camera sensor
- Change type to depth `<sensor type="depth" name="depth">`
- Add `<depth_camera>`  tag

  ```xml
  <camera name="head">
    <depth_camera>
        <output>depths</output>
    </depth_camera>
    ...
  ```

```xml
<gazebo reference="sensor">
    <sensor type="depth" name="depth">
        <update_rate>30.0</update_rate>
        <visualize>1</visualize>
        <pose frame="sensor">${width/2} 0 ${height1/2} 0 0 0</pose>
        <camera name="head">
            <depth_camera>
                <output>depths</output>
            </depth_camera>
            <horizontal_fov>1.3962634</horizontal_fov>
            <image>
                <width>800</width>
                <height>600</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.02</near>
                <far>300</far>
            </clip>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.007</stddev>
            </noise>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>10.0</updateRate>
            <cameraName>camera</cameraName>
            <frameName>camera_frame</frameName>
            <!-- <frameName>camera_rgbd_frame</frameName> -->
            <imageTopicName>/camera/rgb/image_raw</imageTopicName>
            <depthImageTopicName>/camera/depth/image_raw</depthImageTopicName>
            <pointCloudTopicName>/camera/depth/points</pointCloudTopicName>
            <cameraInfoTopicName>/camera/rgb/camera_info</cameraInfoTopicName>
            <depthImageCameraInfoTopicName>/camera/depth/camera_info</depthImageCameraInfoTopicName>
            <pointCloudCutoff>0.4</pointCloudCutoff>
            <hackBaseline>0.07</hackBaseline>
            <distortionK1>0.0</distortionK1>
            <distortionK2>0.0</distortionK2>
            <distortionK3>0.0</distortionK3>
            <distortionT1>0.0</distortionT1>
            <distortionT2>0.0</distortionT2>
            <CxPrime>0.0</CxPrime>
            <Cx>0.0</Cx>
            <Cy>0.0</Cy>
            <focalLength>0.0</focalLength>
        </plugin>
    </sensor>
</gazebo>
```

> Tip: Pointcloud `libgazebo_ros_openni_kinect.so` has wrong tf,
```xml
<joint name="camera_depth_joint" type="fixed">
    <origin xyz="0 0 0" rpy="${-PI/2} 0 ${-PI/2} "/>
    <parent link="camera_link"/>
    <child link="camera_depth_link"/>
</joint>
<link name="camera_depth_link"/>
```

OR  Add tf message to fix it

```xml
<node pkg="tf" 
    type="static_transform_publisher" 
    name="ime_slam_camera_tf" 
    args="0 0 0 -1.58 0 -1.58 /camera/camera_link /camera/camera_depth_link 30"/>
```



## gazebo
![](/images/2019-05-16-12-16-37.png)
## rviz
![](/images/2019-05-16-12-15-44.png)
&nbsp;  
&nbsp;  
&nbsp;  
# References
1. [ROS Robotics projects](https://github.com/qboticslabs/ros_robotics_projects/tree/master/chapter_10_codes/sensor_sim_gazebo)
2. [Gazebo 1.5: libgazebo_openni_kinect.so: pointcloud data is rotated and flipped?](http://answers.gazebosim.org/question/4266/gazebo-15-libgazebo_openni_kinectso-pointcloud-data-is-rotated-and-flipped/)
3. [University of Washington Robotics Capstone](https://github.com/cse481wi18/cse481wi18/wiki)