---
layout: post
title: Gazebo color and texture
categories: gazebo
tags: [sdf]
description: Set color and texture in sdf files
image: terrain.png
public: true
---
# Content
- [Gazebo colors](#colors)
- [Textures and images](#textures)


# Colors
We can set color parameters by define
- ambient
- diffuse
- specular

or using predefined color define in file `/usr/share/gazebo-9/media/materials/scripts/gazebo.material`

for example red color
```
material Gazebo/Red
{
  technique
  {
    pass ambient
    {
      ambient 1 0 0
      diffuse 1 0 0
      specular 0.1 0.1 0.1 1 1
    }
  }
}

```

- Usage
```xml
<link>
...
<visual name='visual'>
    <geometry>
        <box>
            <size>1 1 1</size>
        </box>
    </geometry>
    <material>
        <lighting>1</lighting>
        <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Red</name>
        </script>
    </material>
</visual>
...
</link>
```

## sdf material specification
![](/images/2019-06-02-21-38-41.png)

&nbsp;  
&nbsp;  
&nbsp;  
# Textures
For requested model add folders
- materials/textures 
- materials/scripts

```
├── materials
│   ├── scripts
│   │   └── mybox.material
│   └── textures
│       └── aruco.png
├── model.config
└── model.sdf
```

## mybox.material
```
material mybox/aruco
{
    technique
    {
        pass
        {
            texture_unit
            {
                texture aruco.png
            }
        }
    }
}
```

## model sdf link visual script
```
<link>
...
<visual name='visual'>
    <geometry>
        <box>
            <size>0.1778 0.1778 1e-5</size>
        </box>
    </geometry>
    <material>
        <script>
            <uri>model://mybox/materials/scripts</uri>
            <uri>model://mybox/materials/textures</uri>
            <name>mybox/aruco</name>
        </script>
    </material>
</visual>
...
</link>
```

![](/images/2019-06-02-23-37-56.png)


# reference
- [aruco_gazebo](https://github.com/joselusl/aruco_gazebo)