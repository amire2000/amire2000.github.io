---
layout: post
title: ROS launch files
categories: ROS
tags: [ros, launch]
---

# Arguments and Params in launch files
- Pass arguments to Node
- Pass arguments from parent launch file
- Set rosparam and Node usage

## LAB setup
- Create a package
  - Create a launch folder
  - create a scripts folder
- Create simple python node (sample from ROS wiki)
- Create launch files to simulate scenarios
  - use arguments
  - use rosparam

> # Don't forget run roscore

- Node code sample from ROS wiki
  
```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #message
        hello_str = "hello world arg1:{} arg2:{}".format(
            sys.argv[1], 
            sys.argv[2]) 
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

### Pass arguments to node
  - launch file
    - declare tow arguments
```xml
<?xml version="1.0"?>
<launch>
    <arg name="arg1" default="1"/>
    <arg name="arg2" default="2"/>

    <node name="test" pkg="my_lab" 
        type="pypub.py" 
        output="screen"
        args="$(arg arg1) $(arg arg2)" />
</launch>
```

- Override arguments from command line
```
roslaunch my_lab launch1.launch arg1:=10
```

### Pass arguments from parent launch file
- main_wrapper.launch
```xml
<?xml version="1.0"?>
<launch>
    <arg name="arg2" default="20"/>
    <include file="$(find my_lab)/launch/launch1.launch">
        <arg name="arg1" default="10"/>
        <arg name="arg2" default="$(arg arg2)"/>
    </include>
</launch>
```

## Passing rosparam
ROS parameters types (scope)
- global
- relative
- private

Read param value from code
```python
global_name = rospy.get_param("/global_name")
relative_name = rospy.get_param("relative_name")
private_param = rospy.get_param('~private_name')
```

#### Change Node message build format 

```python
hello_str = "hello world private param :{}, relative   param:{}".format(
    rospy.get_param("~private_param"), 
    rospy.get_param("relative_param")) 
```

- launch file
```xml
<?xml version="1.0"?>
<launch>
<param name="relative_param" value="relative"/>
    <node name="test" pkg="my_robot_slam" 
        type="pypub.py" 
        output="screen">
            <param name="private_param" value="private"/>
    </node>
</launch>
```

## Set param from argument value
```xml
<?xml version="1.0"?>
<launch>
	<arg name="arg1" default="arg_value"/>
	<param name="relative_param" value="relative"/>
	<node name="test" pkg="my_robot_slam" type="pypub.py" output="screen">
		<param name="private_param" value="$(arg arg1)"/>
	</node>
</launch>
```
