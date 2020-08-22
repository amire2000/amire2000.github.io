---
layout: post
title: ROS URDF Part 1
categories: ros
tags: [urdf, xacro, rviz]
image: urdf.png
description: The Universal Robotic Description Format (URDF) is an XML file format used in ROS to describe all elements of a robot
public: true
---

# LAB
- Ubuntu 20.04
- ROS Neotic

# Folders
```
 robot_description
    ├── rviz_config
    ├── config
    ├── launch
    │   └── mrm.launch
    └── urdf
        └── mrm.xacro

```

# urdf / xacro 
- Tow links
- One joint


```xml
<?xml version="1.0" ?>

<robot name="mrm"
    xmlns:xacro="http://www.ros.org/wiki/xacro">
    <link name="base_link">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <box size="1 1 1"/>
            </geometry>
        </visual>
    </link>

    <joint name="base_link__link_01" type="revolute">
        <axis xyz="0 0 1" />
        <limit effort="1000.0" lower="-3.14" upper="3.14" velocity="0.5" />
        <origin rpy="0 0 0" xyz="0 0 0.5"/>
        <parent link="base_link"/>
        <child link="link1"/>
    </joint>
    <link name="link1">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0.2"/>
            <geometry>
                <!-- <cylinder radius="0.35" length="0.4"/> -->
                <box size="0.2 0.3 0.5"/>
            </geometry>
        </visual>
    </link>
</robot>
```

## Check
- With `check_urdf`
- With `urdf_to_graphiz`

> View `urdf_to_graphiz` with `xdg-open`

![](/images/2020-08-19-21-40-59.png)


# launch

```
<launch>
    <param name="robot_description" 
        command="$(find xacro)/xacro --inorder '$(find robot_description)/urdf/mrm.xacro'"/>
    <node pkg="robot_state_publisher" 
          name="robot_state_publisher" 
          type="robot_state_publisher" />
    <node name="rviz" pkg="rviz" type="rviz"/>
    
    <node name="joint_state_publisher" 
          pkg="joint_state_publisher_gui" 
          type="joint_state_publisher_gui" >
    </node>
</launch>
```

> Changes from previous versions
> - joint_state_publisher and `use_gui` move to `joint_state_publisher_gui`
> - `state_publisher` type in pkg `robot_state_publisher` move to type `robot_state_publisher`

# Rviz

- Add `RobotModel`
- Change Fixed frame to `base_link`
  
![](/images/2020-08-18-21-26-53.png)

## Save and reuse config
- Save config to `rviz_config` folder
- Update launch file `rviz` node to load config

```xml
<node pkg="rviz" 
      name="rviz" 
      type="rviz" 
      args="-d $(find robot_description)/rviz_config/rviz.config.rviz" />
```