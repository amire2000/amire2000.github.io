---
layout: post
title: Set link velocity
categories: gazebo
tags: [link, velocity]
public: true
description: Set Link velocity
---
# Content
- [Set velocity instantaneously](#set-velocity-instantaneously)
- Set Link Velocity Using PID controllers

# Set velocity instantaneously
Objects move at the target speed without any forces or torques being applied

- Linear velocity set by method `Link::SetLinearVelocity()` (m/sec)
- Angular  velocity set by method `Link::SetAngularVelocity()` (rad/sec)

Both method get 3d vector (x, y, z), linear velocity unit in (m/sec) and angular velocity as (rad/sec)
&nbsp;  
&nbsp;  
&nbsp;  
### Demo (linear velocity)
#### velocity_demo.world
```xml
<?xml version="1.0" ?>
<sdf version="1.5">
	<world name="default">
		<gravity>0 0 0</gravity>
		<!-- A global light source -->
		<include>
			<uri>model://sun</uri>
		</include>
		<!-- A ground plane -->
		<include>
			<uri>model://ground_plane</uri>
		</include>
		<model name='link_velocity_demo'>
			<plugin name='set_link_velocity_plugin' filename='libmy_link_vel.so'/>
			<link name='white_link_0'>
				<pose frame=''>0 0 1 0 0 0</pose>
				<visual name='white_visual_0'>
					<geometry>
						<cylinder>
							<radius>0.1</radius>
							<length>0.5</length>
						</cylinder>
					</geometry>
					<material>
						<lighting>1</lighting>
						<script>
							<uri>file://media/materials/scripts/gazebo.material</uri>
							<name>Gazebo/White</name>
						</script>
					</material>
				</visual>
			</link>
		</model>
		
	</world>
</sdf>
```
> Set gravity to zero for model simplicity `<gravity>0 0 0</gravity>`, tag are `world` child

#### model_velocity.cpp
```cpp
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>
#include <math.h>

namespace gazebo
{
class ModelMove : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->model = _parent;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelMove::OnUpdate, this));
  }

  void OnUpdate()
  {
    model->GetLink("white_link_0")->SetLinearVel({0, 1, 0});
  }

private:
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelMove)
} // namespace gazebo
```

- meson.build
```python
gz = dependency('gazebo')
install_dir = meson.source_root() + '/bin'

sources = ['model_move/model_move.cc']
myplug = library('my_link_vel', sources: sources,
        dependencies : [gz],
        install: true,
        install_dir: [install_dir])
```

#### build and install
```
ninja -C build install
```

#### run
```bash
gazebo --verbose -u velocity_demo.world
```

![](/images/set_linear_velocity.gif)
# Reference
- [Setting Velocity on Links And Joints](http://gazebosim.org/tutorials?tut=set_velocity&cat=)

