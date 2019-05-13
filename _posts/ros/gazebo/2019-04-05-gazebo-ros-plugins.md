---
layout: post
title: Gazebo ROS sensors
categories: ROS
tags: [ros, gazebo, sensors, plugin]
---

# Objectives
- Add model to world
  - Using 
- Integrated Gazebo model sensor with ROS using plugin
- View Plugin and Sensor output
  - rviz
  - gazebo topic viewer


# Project
```
── launch
│   └── kinect.launch
├── models
│   └── kinect
│       ├── materials
│       │   └── textures
│       │       └── kinect.png
│       ├── meshes
│       │   └── kinect.dae
│       ├── model.config
│       └── model.sdf
├── src
└── worlds
    └── kinect.world

```

## Depth camera
### Kinect
- Download model and place it in `models` folder
#### Spawn model
- Spawn from file
```
rosrun gazebo_ros spawn_model \
-file `rospack find gazebo_integration`/models/kinect/model.sdf \
-sdf \
-model kinect \
-x 0 -y -1 -z 0 \
-Y 1.57 -P -0.2
```
- Spawn from database
```
rosrun gazebo_ros spawn_model \
-database cube_20k \
-sdf \
-model box1 \
-x 1 -y -1 -z 0
```

#### Gazebo axis and orientation
![](/images/2019-05-04-11-09-11.png)
![](/images/2019-05-04-13-14-37.png)

#### launch
- Set environment `GAZEBO_MODEL_PATH` in launch file

```xml
<?xml version="1.0"?>
<launch>
	<env name="GAZEBO_MODEL_PATH" value="$(find gazebo_integration)/models:$GAZEBO_MODEL_PATH" />
	<include file="$(find gazebo_ros)/launch/empty_world.launch">
		<arg name="world_name" default="worlds/kinect.world"/>
        
	</include>
</launch>
```

![](/images/2019-05-04-13-04-33.png)

#### world file (kinect.world)
```xml
<sdf version='1.4'>
	<world name='default'>
		<!-- A global light source -->
		<include>
			<uri>model://sun</uri>
		</include>
		<!-- A ground plane -->
		<include>
			<uri>model://ground_plane</uri>
		</include>
		<include>
			<static>true</static>
			<uri>model://cube_20k</uri>
			<pose>0.5 2.5 0 0 0 0</pose>
		</include>
		<include>
			<static>true</static>
			<uri>model://coke_can</uri>
			<pose>-0.3 1 0 0 0 0</pose>
		</include>
		<include>
			<static>true</static>
			<uri>model://kinect</uri>
            <name>kinect_model</name>
			<pose>0 0 0 0 0 1.57</pose>
		</include>
	</world>
</sdf>
```

### Gazebo topics
```bash
gz topic --list

/gazebo/default/atmosphere
/gazebo/default/default/camera/cmd
/gazebo/default/diagnostics
/gazebo/default/factory
/gazebo/default/factory/light
/gazebo/default/gui
/gazebo/default/gzclient_camera/cmd
/gazebo/default/joint
/gazebo/default/kinect_model/link/camera/image
.....

```

# ROS
- Add plugin to model sensor
- Launch gazebo again
```
/camera/depth/camera_info
/camera/depth/image_raw
/camera/depth/image_raw/compressed
/camera/depth/image_raw/compressed/parameter_descriptions
/camera/depth/image_raw/compressed/parameter_updates
/camera/depth/image_raw/compressedDepth
/camera/depth/image_raw/compressedDepth/parameter_descriptions
/camera/depth/image_raw/compressedDepth/parameter_updates
/camera/depth/image_raw/theora
/camera/depth/image_raw/theora/parameter_descriptions
/camera/depth/image_raw/theora/parameter_updates
/camera/depth/points
/camera_ir/depth/camera_info
/camera_ir/parameter_descriptions
/camera_ir/parameter_updates

```
## View in rviz
> Set rviz Fixed frame as Plugin `<frameName>` in the code example `camera_frame`

Add to view
- img rgb: `/camera/rgb/image_raw`
- img depth: `/camera/depth/image_raw`
- PointCloud2: `/camera/depth/points`
![](/images/2019-05-05-10-56-47.png)

# sensor plugin code
```xml
<sensor type="depth" name="openni_camera">
	<always_on>true</always_on>
	<update_rate>20.0</update_rate>
	<visualize>true</visualize>
	<camera>
		<horizontal_fov>1.047</horizontal_fov>
		<image>
			<width>640</width>
			<height>480</height>
			<format>R8G8B8</format>
		</image>
		<depth_camera></depth_camera>
		<clip>
			<near>0.1</near>
			<far>20</far>
		</clip>
	</camera>
	<plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
		<alwaysOn>true</alwaysOn>
		<updateRate>10.0</updateRate>
		<cameraName>camera</cameraName>
		<frameName>camera_frame</frameName>
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
```
