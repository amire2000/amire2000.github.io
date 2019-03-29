---
layout: post
title: VSCode config and setting for ROS
categories: vscode
tags: [ros, vscode]
---

## Extensions
### urdf
- Associates .urdf/.xacro files with xml format.
- Some snippets

![](/images/2018-12-07-09-48-36.png)

## Settings.js

### file association
~~~json
"files.associations": {
    ".sdf": "xml",
    ".world": "xml"
},
~~~

### tasks.json
- config `catkin` tasks

### build
- config use to extend workspace
- build with debug symbols
```json
{
    "label": "build",
    "type": "shell",
    "command": "catkin config --extend /opt/ros/melodic && catkin build -DCMAKE_BUILD_TYPE=Debug -j4"
}
```