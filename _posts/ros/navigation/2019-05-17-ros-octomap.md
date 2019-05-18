---
layout: post
title: octomap basic usage
categories: ros
tags: [navigation, octomap]
image: octomap_tree.png
public: true
description: Install and Run octomap mapping , create a map save it and load
---
# Content
- Install
- Mapping
- Provide a map
  
# Install
```
sudo apt install ros-melodic-octomap-mapping
```

# Mapping
- Config and Launch `octomap_server`
- remap
  - frame_id to odom
  - cloud_in to PointCloud2 published topic
  
```xml
<launch>
	<node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
		<param name="resolution" value="0.05" />
		
		<!-- fixed map frame (set to 'map' if SLAM or localization running!) -->
		<param name="frame_id" type="string" value="odom" />
		
		<!-- maximum range to integrate (speedup!) -->
		<param name="sensor_model/max_range" value="5.0" />
		
		<!-- data source to integrate (PointCloud2) -->
		<remap from="cloud_in" to="/camera/depth/points" />
	
	</node>
</launch>
```

- Run rviz
  - Add MarkerArray
    - topic: /occupied_cells_vis_array

- Run  teleop and create a map
  
![](/images/2019-05-17-16-15-31.png)

## Save
```
rosrun octomap_server octomap_saver -f maps/my_octomap.bt
```

# Provide a map
> Stop octomap mapping if run
```
rosrun octomap_server octomap_server_node maps/my_octomap.bt
```

- Open rviz again
  - Add MarkerArray Display and select `occupied_cells_vis_array` to view provided map

# octovis

```
sudo apt-get install ros-melodic-octovis
```

# Reference
- [Octomap explanierend](http://ros-developer.com/2017/11/27/octomap-explanierend/)