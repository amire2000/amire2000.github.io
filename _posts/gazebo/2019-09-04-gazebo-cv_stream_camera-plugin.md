---
layout: post
title: Gazebo stream camera plugin
categories: gazebo
tags: [opencv, plugin, meson]
description: Using opencv to capture image frame from gazebo encode as jpeg and send it over udp stream, build plugin code using meson build system
image: plug.png
public: true
---

```python
incdir = include_directories('/usr/include/OGRE', '/usr/include/OGRE/Paging')
gz = dependency('gazebo')
cv = dependency('opencv')
gz_plugin_dirs = '/usr/lib/x86_64-linux-gnu/gazebo-9/plugins'
gz_plugs = meson.get_compiler('cpp', cross : false).find_library('CameraPlugin', dirs: [gz_plugin_dirs])

install_dir = meson.source_root() + '/bin'

mylib = library('mylib', sources : ['cv_camera.cpp'],
                dependencies : [gz, cv, gz_plugs],
                include_directories: [incdir],
                install: true,
                install_dir: [install_dir])
```