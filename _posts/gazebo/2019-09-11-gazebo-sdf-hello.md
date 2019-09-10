---
layout: post
title: SDF hello
categories: gazebo
tags: [sdf]
description: SDF links joints hello worlds
public: true
image: sdf.png
---

# First model
- Add simple model into empty world
- Both world and model are declared in SDF format
&nbsp;  
&nbsp;  
&nbsp;  
- t1.world
  
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
	<world name="default">
		<include>
			<uri>model://sun</uri>
		</include>
		<include>
			<uri>model://ground_plane</uri>
		</include>
        <!--simple model-->
		<model name='m1'>
			<pose>0 0 0.05 0 0 0</pose>
			<link name='l1'>
				<visual name='visual'>
					<geometry>
						<box>
							<size>0.1 0.1 0.1</size>
						</box>
					</geometry>
				</visual>
				<collision name="collision">
					<geometry>
						<box>
							<size>0.1 0.1 0.1</size>
						</box>
					</geometry>
				</collision>
			</link>
		</model>
	</world>
</sdf>
```

- Run it

```
gazebo --verbose t1.world
```

![](/images/2019-09-11-00-21-03.png)

# Add link and joint

- View -> Transparent
- View -> Joints
  
![](/images/2019-09-11-00-35-45.png)
## Joint position
- joint position in respect to child center of mess


```xml
<joint name="l1_l2_joint" type="revolute">
    <parent>l1</parent>
    <child>l2</child>
    <pose>0 0 0 0 0 0</pose>
    <axis>
        <xyz>0 0 1</xyz>
    </axis>
</joint>
```
## Link position
- Link pose respect to model `pose`
- Position measure from the center of mess

![](/images/2019-09-11-00-45-53.png)

```xml
<link name='l2'>
    <pose>0 0 0.1 0 0 0</pose>
    <visual name='visual'>
        <geometry>
            <box>
                <size>0.1 0.1 0.1</size>
            </box>
        </geometry>
```
- View -> Center of Mass
  
![](/images/2019-09-11-00-38-31.png)

# Apply Force/Torque
- Select link
- Right click and chose `Applay Force / Torque`
  
![](/images/gazebo_terque.gif)

#  Example SDF
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
	<world name="default">
		<include>
			<uri>model://sun</uri>
		</include>
		<include>
			<uri>model://ground_plane</uri>
		</include>
		<!--simple model-->
		<model name='m1'>
			<pose>0 0 0.05 0 0 0</pose>
			<link name='l1'>
				<visual name='visual'>
					<geometry>
						<box>
							<size>0.1 0.1 0.1</size>
						</box>
					</geometry>
				</visual>
				<collision name="collision">
					<geometry>
						<box>
							<size>0.1 0.1 0.1</size>
						</box>
					</geometry>
				</collision>
			</link>
			<joint name="l1_l2_joint" type="revolute">
				<parent>l1</parent>
				<child>l2</child>
				<pose>0 0 0 0 0 0</pose>
				<axis>
					<xyz>0 0 1</xyz>
				</axis>
			</joint>
			<link name='l2'>
				<pose>0 0 0.1 0 0 0</pose>
				<visual name='visual'>
					<geometry>
						<box>
							<size>0.1 0.1 0.1</size>
						</box>
					</geometry>
					<material>
						<lighting>1</lighting>
						<script>
							<uri>file://media/materials/scripts/gazebo.material</uri>
							<name>Gazebo/Red</name>
						</script>
					</material>
				</visual>
				<collision name="collision">
					<geometry>
						<box>
							<size>0.1 0.1 0.1</size>
						</box>
					</geometry>
				</collision>
			</link>
		</model>
	</world>
</sdf>
```

# Joint between models