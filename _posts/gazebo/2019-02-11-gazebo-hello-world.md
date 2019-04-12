---
layout: post
title: Gazebo hello world
categories: gazebo
tags: [gazebo, worlds, ruby, spawn]
---
- Gazebo worlds
- Running gazebo
- Gazebo model
- Spawn model into world
- Using Ruby (erb)

# Gazebo worlds
world files are sdf files that contains other modules
and world settings like Physics Properties

SDF is an XML format that describes objects and environments for robot simulators, visualization, and control. 

> Note: Gazebo work with different physics engine the default are ODE (Open Dynamics Engine)
>
- empty.world
```xml
<?xml version="1.0"?> 
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>
</world>
</sdf>
```

# Running gazebo
```
gazebo --verbose -u <world path>/empty.world
```
- verbose
- u: start gazebo in pause 

![](images/2019-04-05-10-54-57.png)

### Running as separated
-  gzserver
-  gzclient

### Demo
- Terminal1
```
gzserver --verbose -u <world path>/empty.world
```
- Terminal2
```
gzclient
```

# Gazebo model
Gazebo is able to dynamically load models into simulation either programmatically or through the GUI.


Models are `sdf` files define a physical entity with dynamic, kinematic, and visual properties. In addition, a model may have one or more plugins, which affect the model's behavior. A model can represent anything from a simple shape to a complex robot; even the ground is a model. 

- environment variable : `GAZEBO_MODEL_PATH`
- model are set in
  - `/usr/share/gazebo-9/models`
  - <home-directory>/.gazebo/models

> Set env. variables by sourcing
```
source /usr/share/gazebo/setup.sh
```

# Spawn model into world
Insert model from command line
```
#from models folder
gz model --spawn-file  mybox/model.sdf -m ox -x -1.0 -y 1.0 -z 0 -R 0 -P 0 -Y 0
```

-m: model name
--spawn-file: sdf location
-x, -y, -z
-R, -P, -Y

> -p option not working

# Using Ruby (erb)
Using ruby erb (embedded ruby) template engine

> run erb template as separate command

> erb installed as part of ruby install
> `sudo apt install ruby2.5`
```
$ erb world.sdf.erb > world.sdf
$ gazebo world.sdf
```


# Reference
- [An Introduction to ERB Templating](http://www.stuartellis.eu/articles/erb/)
- [3dwarehouse](https://3dwarehouse.sketchup.com)