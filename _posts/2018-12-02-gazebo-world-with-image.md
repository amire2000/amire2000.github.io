---
layout: post
title: How to build a world with real image as ground plane
categories: gazebo
tags: [gazebo]
---

Add gazebo world ground with special pattern

> This ground will be use in "follow the line" scenario 

### Ground model folder and files structure
~~~
├── models
│   └── my_ground_plane
│       ├── materials
│       │   ├── scripts
│       │   │   └── my_ground_plane.material
│       │   └── textures
│       │       └── course.png
│       ├── model.config
│       └── model.sdf
└── worlds
    └── my.world
~~~

### file: models/my_ground_plane/materials/scripts/my_ground_plane.material
~~~
material MyGroundPlane/Image
{
  technique
  {
    pass
    {
      ambient 0.5 0.5 0.5 1.0
      diffuse 1.0 1.0 1.0 1.0
      specular 0.0 0.0 0.0 1.0 0.5

      texture_unit
      {
        texture course.png
      }
    }
  }
}
~~~

### file: models/my_ground_plane/model.config
~~~xml
<?xml version="1.0" encoding="UTF-8"?>
<model>
   <name>My Ground Plane</name>
   <version>1.0</version>
   <sdf version="1.4">model.sdf</sdf>
   <description>My textured ground plane.</description>
</model>
~~~

### file: models/my_ground_plane/model.sdf
~~~xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.4">
   <model name="my_ground_plane">
      <static>true</static>
      <link name="link">
         <collision name="collision">
            <geometry>
               <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
               </plane>
            </geometry>
            <surface>
               <friction>
                  <ode>
                     <mu>100</mu>
                     <mu2>50</mu2>
                  </ode>
               </friction>
            </surface>
         </collision>
         <visual name="visual">
            <cast_shadows>false</cast_shadows>
            <geometry>
               <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
               </plane>
            </geometry>
            <material>
               <script>
                  <uri>model://my_ground_plane/materials/scripts</uri>
                  <uri>model://my_ground_plane/materials/textures</uri>
                  <name>MyGroundPlane/Image</name>
               </script>
            </material>
         </visual>
      </link>
   </model>
</sdf>
~~~


### world
- Add include to world
  
~~~xml
<include>
    <uri>model://my_ground_plane</uri>
</include>
~~~

## Run
- export `GAZEBO_MODEL_PATH` to custom models path
- run: `gazebo --verbose worlds/my.world`

![](/images/2018-12-02-22-16-33.png)
## Reference 
- [Gazebo Q&A](http://answers.gazebosim.org/question/4761/how-to-build-a-world-with-real-image-as-ground-plane/)