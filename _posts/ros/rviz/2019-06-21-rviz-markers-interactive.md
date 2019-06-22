---
layout: post
title: RVIZ Interactive Markers
categories: ros
tags: [rviz, markers]
image: markerII.jpeg
description: Run RVIZ interactive markers to control and send message back from rviz control to your robots
public: true
---
# Content
- Basic Interactive marker
- Code explain (python)
- Usage
  
Interactive markers are similar to the "regular" markers, however they allow the user to interact with them by changing their position or rotation, clicking on them or selecting something from a context menu assigned to each marker.

For interactive markers we need to instantiate an `InteractiveMarkerServer` object. This will handle the connection to the client (usually RViz) and make sure that all changes we make are being transmitted and that your application is being notified of all the actions the user performs on the interactive markers. 

![](/images/2019-06-22-14-29-40.png)

![](/images/2019-06-21-18-54-21.png)
image from ROS wiki

&nbsp;  
&nbsp;  
&nbsp;  
# Code
- Declare interactive marker server
- Add box marker
- Assign box control to interactive marker
- Attach interactive server callback
- Run ROS loop and apply changes to the server  

{% gist 35f7cfbae5b3b25c833c3ad105864f0c %}


&nbsp;  
&nbsp;  
&nbsp;  
# Usage
- Terminal 1

```bash
roscore
```

- Terminal 2
> Don't forget to add executable permission
```bash
rosrun my_rviz_markers interactive_marker.py
```

- Terminal 3
  - Run rviz
  - Set `Fixed Frame` to `base_link`
  - Add `InteractiveMarkers`
    - set `Marker Topic` to `simple_marker/my_marker`

```bash
rosrun rviz rviz
```
![](/images/2019-06-21-19-34-17.png)

![](/images/2019-06-22-08-36-10.png)



# Reference
- [Interactive Markers: Getting Started](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started)
- [Interactive Markers: Writing a Simple Interactive Marker Server](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Writing%20a%20Simple%20Interactive%20Marker%20Server)
- [Interactive markers tutorials and documentation](https://github.com/cse481sp17/cse481c/wiki/Lab-13:-Creating-Interactive-Visualizations-in-RViz-using-InteractiveMarkers)