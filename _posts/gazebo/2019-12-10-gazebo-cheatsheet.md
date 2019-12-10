---
layout: post
title: Gazebo tips tricks and cheat
categories: gazebo
tags: [cheat]
description: Gazebo cheat sheet sdf, urdf and xacro notes
public: true
image: gazebo.png
---

# Insert model
- Run gazebo
  
```bash
# minimal
gz model --spawn-file=<urdf_file> --model-name=<model name>
```

# Convert xacro to urdf
```
xacro skid_steer.xacro.urdf > skid_steer.urdf
```

# check urdf
```
check_urdf ss.urdf
```