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