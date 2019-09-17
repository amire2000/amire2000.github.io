---
layout: post
title: Revolute joint and PID controller
categories: gazebo
tags: [pid, joint]
public: true
---
# Content
- Create module with revolute joint
- Add plugin 
  - PID set position

## Model
> base on [Gazebo Newsletter 12 August 2018](http://gazebosim.org/blog)

{% gist 11243fd8176c214aa0b4df11b4da0f47 %}

## Plugin
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
    gzmsg << "hello\n";
    gzmsg << "hello\n";
    this->model = _parent;
    this->lastUpdateTime = this->model->GetWorld()->SimTime();
    this->pid.Init(1, 0, 0, 0, 0, 1.0, -1.0);
    this->joint = this->model->GetJoint("revolute_demo");
    if (!this->joint)
    {
      gzwarn << "joint not found\n";
    }
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
      double error = angle - 0.785398;

      double force = this->pid.Update(error, dt);
      // gzmsg << "force:" << force << "\n";
      this->joint->SetForce(0, force);
      this->lastUpdateTime = time;
    }
  }

private:
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection;
  physics::JointPtr joint;
  common::Time lastUpdateTime;
  common::PID pid;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelMove)
} // namespace gazebo
```

# Appendix
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
	<world name="default">
		<include>
			<uri>model://sun</uri>
		</include>
		<include>
			<uri>model://ground_plane</uri>
		</include>

		<include>
			<uri>model://revolute1</uri>
		</include>
	</world>
</sdf>
```
