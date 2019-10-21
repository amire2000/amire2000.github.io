---
layout: post
title: Gazebo environment variable
categories: gazebo
tags: [env, ]
description: Gazebo environment variable and model loading
image: gazebo.png
public: true
---

# Environment Variables
The default env. are declared

```bash
# source <install_path>/share/gazebo/setup.sh
source /usr/share/gazebo/setup.sh
```

- **GAZEBO_PLUGIN_PATH:** colon-separated set of directories where Gazebo will search for the plugin shared libraries at runtime.
- **GAZEBO_MODEL_PATH:** colon-separated set of directories where Gazebo will search for models
- **GAZEBO_RESOURCE_PATH:** colon-separated set of directories where Gazebo will search for other resources such as world and media files.