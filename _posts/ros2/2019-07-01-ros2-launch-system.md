---
layout: post
title: ROS2 Launch system
categories: ros2
tags: [launch]
description: ROS2 Launch system tutorial
public: true
image: Launch.jpg
---
#  Content
- Simple package with python node and launch file
- Launch life cycle

&nbsp;  
&nbsp;  
# Init
- Create pkg in ros2_ws
- Add `launch` folder
  
```bash
# from ros2_ws
ros2 pkg create ros2_py
# from pkg folder
mkdir launch
```
&nbsp;  
&nbsp;  
# Simple node (python)
> Don't forget to set execution permission


```python
#!/usr/bin/env python3
import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("py_pub")
    node.get_logger().info("py pub simple")
    
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```
&nbsp;  
&nbsp;  
# Basic launch file
> Don't forget to set execution permission

```python
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package="ros2_py",
            node_executable="pub.py",
            output="screen"
        )
    ])
```

&nbsp;  
&nbsp;  
# CMakeLists 
- Add rule for node source file
- Add rule for launch file
- > Don't forget to set execution permission for both files

- script / node rules
  - copy node source file to install workspace (using `PROGRAM` keyword to maintain permission)


```
install(PROGRAMS src/pub.py 
  DESTINATION lib/${PROJECT_NAME})
```

- launch files rule
  - copy all launch files to install workspace
  
```
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```

&nbsp;  
&nbsp;  
# build & Usage
> All actions run from `ros2_ws` root folder
- run `colcon` 
- from `ws` folder source `setup.bash`
- run node


```bash
# build
colcon build --symlink-install
#Source env
source install/setup.bash
#Run launch
ros2 launch ros2_py simple.launch.py
```

```
[INFO] [launch]: process[pub.py-1]: started with pid [19796]
[INFO] [py_pub]: py pub simple
[INFO] [launch]: process[pub.py-1]: process has finished cleanly

```
&nbsp;  
&nbsp;  
# Reference
- [How to create Launch Files in ROS2](https://www.youtube.com/watch?v=vsH6dNXTcss)