---
layout: post
title: Control joint 
categories: gazebo
tags: [sdf, joint, velocity, pid, force, position]
description: Basic example show how to control joint with force, velocity and position with gazebo plugin
public: true
image: gjoint.png
---
# Content
- Velocity
  - instant velocity
  - pid velocity
- Position
  - instant position
- Force
  - pid

&nbsp;  
&nbsp;  
&nbsp;  
# Joint velocity

- Set Instantaneous Velocity

## Set Instantaneous Velocity
- Simple, only one function call
- Object moves at target velocity right away
- It must be set every time step to keep the object at a constant velocity forever.
  
## Implement
- Add to plugin `update` method

```cpp
//SetVelocity(axes, velocity) 
//axes: 0/1 , prismatic:1 revolute:1 revolute2:2 screw:1 universal:1
//velocity: SI units (rad/s or m/s)
this->model->GetJoint("b2c")->SetVelocity(0, 1.0);
```

## PID Controller

![](/images/vel-joint.gif)

### using joint controller
```c
void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->model = _parent;
    this->pid = common::PID(1, 0, 0);
    this->joint = this->model->GetJoint("b2c");
    this->model->GetJointController()->SetVelocityPID(
    this->joint->GetScopedName(), this->pid);
    this->model->GetJointController()->SetVelocityTarget(
    this->joint->GetScopedName(), 5.0);
    
  }
```

&nbsp;  
&nbsp;  
&nbsp;  
# Joint Position
- Add to plugin `update` method

```cpp
//SI units: Newtons
this->model->GetJoint("b2c")->SetPosition(0, 0.707);
```
&nbsp;  
&nbsp;  
&nbsp;  
# Joint Force
- Control position using PID controller 
  
> Force is additive
- Add to plugin `update` method

```cpp
//
//
this->model->GetJoint("b2c")->SetForce(0, 100);
```

## PID Example
- Update `Load` and `Update` plugin method
  
```cpp
void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->model = _parent;
    this->lastUpdateTime = this->model->GetWorld()->SimTime();
    this->pid.Init(1, 0, 0, 0, 0, 1.0, -1.0);
    this->joint = this->model->GetJoint("b2c");
    
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelMove::OnUpdate, this));
  }

  void OnUpdate()
  {
    double angle = this->joint->Position(0);
    
    common::Time time = this->model->GetWorld()->SimTime();
    if (time < this->lastUpdateTime)
    {
      this->lastUpdateTime = time;
      return;
    }
    else if (time > this->lastUpdateTime)
    {
      double dt = (this->lastUpdateTime - time).Double();
      double error = angle - 1.57;

      double force = this->pid.Update(error, dt);
      // gzmsg << "force:" << force << "\n";
      this->joint->SetForce(0, force);
      this->lastUpdateTime = time;
    }
  }
```

### Run demo
- Add plot `window-plot`
  - select and drag variables to plot  ``

![](/images/2019-09-21-17-21-12.png)

- Insert model with pid and position force control
  
![](/images/joint_pid.gif)

&nbsp;  
&nbsp;  
&nbsp;  
# Test Model
```xml
<?xml version='1.00'?>
<sdf version='1.6'>
	<model name="test">
		<pose>0 0 0.05 0 0 0</pose>
        <joint name="world_fix" type="fixed">
			<parent>world</parent>
			<child>cylinder</child>
		</joint>
		<joint name="b2c" type="revolute">
			<parent>cylinder</parent>
			<child>box</child>
			<axis>
				<xyz>0 0 1</xyz>
				<dynamics>
					<damping>0.5</damping>
				</dynamics>
			</axis>
		</joint>
		<link name='cylinder'>
			<pose>0 0 0 0 0 0</pose>
			<collision name='collision'>
				<geometry>
					<cylinder>
						<radius>0.1</radius>
						<length>0.1</length>
					</cylinder>
				</geometry>
			</collision>
			<visual name='visual'>
				<geometry>
					<cylinder>
						<radius>0.1</radius>
						<length>0.1</length>
					</cylinder>
				</geometry>
				<material>
					<lighting>1</lighting>
					<script>
						<uri>file://media/materials/scripts/gazebo.material</uri>
						<name>Gazebo/Green</name>
					</script>
				</material>
			</visual>
		</link>
		<link name='box'>
			<pose>0 0 0.1 0 0 0</pose>
			<collision name='collision'>
				<geometry>
					<box>
						<size>0.1 0.1 0.1</size>
					</box>
				</geometry>
			</collision>
			<visual name='visual'>
				<geometry>
					<box>
						<size>0.1 0.1 0.1</size>
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
		</link>
		<plugin name="model_plugin" filename="libmy_joint_vel.so"/>
	</model>
</sdf>
```
&nbsp;  
&nbsp;  
&nbsp;  
# Basic Plugin
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
    
  }

private:
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection;
  physics::JointPtr joint;
  
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelMove)
}

```
&nbsp;  
&nbsp;  
&nbsp;  
# meson build
```python
project('gazebo_tutorial', 'cpp',
    version : '0.1',
    default_options : ['warning_level=3', 'cpp_std=c++11'])
gz = dependency('gazebo')
install_dir = meson.source_root() + '/bin'
sources = ['joint_ctl/joint_ctl.cc']
myplug = library('my_joint_vel', sources: sources,
        dependencies : [gz],
        install: true,
        install_dir: [install_dir])
```
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Setting Velocity on Links And Joints](http://gazebosim.org/tutorials?tut=set_velocity&cat=)
- [SetPoistion](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Joint.html#ad02e021e1745f416f45e13024925714a)