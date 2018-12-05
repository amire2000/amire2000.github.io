---
layout: post
title: Creating Terrain maps for Gazebo
categories: gazebo
tags: [gazebo, sdf]
---

In order to have more realistic environments for robotics scenarios, Gazebo is capable of using 3D terrain heightmaps created from greyscale images.

## Reference
- [Creating Heightmaps for Gazebo](https://github.com/AS4SR/general_info/wiki/Creating-Heightmaps-for-Gazebo)
- [Gazebo: Heightmap Tutorial](https://vimeo.com/58409707)

## Tutorial
- Create gray image
- The height and width must be equal 
- The size of a side must satisfy pixels = 2^n+1 (129*129)
- The image must be uint8


### Create image with gimp
![](/images/2018-12-05-23-36-53.png)

Hight map interpreter each pixel as a hight value
- black: low area
- white: hight area

#### Paint
- Start with black image and add white are

![](/images/2018-12-05-23-43-54.png)

- Add white ares

![](/images/2018-12-05-23-46-00.png)

- Remove alfa channel (layer dialog box) and save (export as png)


### Create a model
- Under project models folder add `hightmap` sub folder
- Add `model.onfig` and `model.sdf`
- Create subfolder `materials/textures` under `hightmap`
- Copy png file to `materials/textures`
  

#### model.sdf
```xml
<?xml version="1.0" ?>
<sdf version="1.5">
    <model name="heightmap">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>model://hightmap/materials/textures/hightmap.png</uri>
              <size>200 200 10</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual_abcedf">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <texture>
                <diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <texture>
                <diffuse>file://media/materials/textures/grass_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <texture>
                <diffuse>file://media/materials/textures/fungus_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <blend>
                <min_height>2</min_height>
                <fade_dist>5</fade_dist>
              </blend>
              <blend>
                <min_height>4</min_height>
                <fade_dist>5</fade_dist>
              </blend>
              <uri>model://hightmap/materials/textures/hightmap.png</uri>
              <size>200 200 10</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
</sdf>
```

```xml
<geometry>
<heightmap>
    <uri>model://hightmap/materials/textures/hightmap.png</uri>
    <size>200 200 10</size>
    <pos>0 0 0</pos>
</heightmap>
</geometry>
```

> Size don't need to match the image size: width length high (max)

#### World file
```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://hightmap</uri>
    </include>
    <physics name='default_physics' default='0' type='ode'>
      <gravity>0 0 -9.8066</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
      <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    </physics>
  </world>
</sdf>
```


#### Run
- Add models folder to environment variable `GAZEBO_MODEL_PATH`
- Run `gazebo worlds/terran.world`
  
![](/images/2018-12-06-00-22-07.png)

