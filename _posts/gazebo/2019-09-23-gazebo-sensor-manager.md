---
layout: post
title: Gazebo Sensor manager demo
categories: gazebo
tags: [sensor, imu]
description: Using sensor manager to read sensor data
public: true
---

# Model
- Base on model from [gazebo joint controller](/gazebo-joint-control/)
- Add imu sensor to box link

```xml
<model>
    <link>
        ...
        <sensor name="imu_sensor" type="imu">
				<pose>0 0 0 0 0 0</pose>
				<always_on>1</always_on>
				<update_rate>10.0</update_rate>
        </sensor>
    </link>
</model>
```

# Plugin
- Plugin usage sensor manager to get and read imu sensor data

```cpp
 #include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>
#include <sensors/sensors.hh> // sensor manager
#include <math.h>

namespace gazebo
{
class ModelMove : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->model = _parent;
    
    this->pid = common::PID(1, 0, 0);
    this->joint = this->model->GetJoint("b2c");
    this->model->GetJointController()->SetVelocityPID(
        this->joint->GetScopedName(), this->pid);
    this->model->GetJointController()->SetVelocityTarget(
        this->joint->GetScopedName(), this->SPEED);

    //init sensor manager
    this->sensor_mgr = sensors::SensorManager::Instance();
    sensors::SensorPtr pSensor = this->sensor_mgr->GetSensor("imu_sensor");
     if (pSensor == nullptr)
    {
      gzerr<<"fail get sensor\n";
    }
    this->pImuSensor = std::dynamic_pointer_cast<sensors::ImuSensor, sensors::Sensor>(pSensor);
    if (pImuSensor == nullptr)
    {
      gzerr<<"fail load imu data\n";
    }
    
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelMove::OnUpdate, this));
  }

  void OnUpdate()
  {
      ignition::math::v4::Quaterniond o = pImuSensor->Orientation();
      gzmsg << o.Roll() << "," << o.Pitch() << ","<< o.Yaw() <<  "\n";
  }

private:
  sensors::ImuSensorPtr pImuSensor;
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection;
  physics::JointPtr joint;
  common::PID pid;
  sensors::SensorManager *sensor_mgr;
  const int SPEED = 0.5;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelMove)
} // namespace gazebo
```

# meson.build
```python
#include point to base folder for sensors
incdir = include_directories('/usr/include/gazebo-9/gazebo')
gz = dependency('gazebo')

install_dir = meson.source_root() + '/bin'

sources = ['model_move/model_move.cc']
myplug = library('my_joint_vel', sources: sources,
        dependencies : [gz],
        include_directories: [incdir],
        install: true,
        install_dir: [install_dir])
```


# Demo
- imu show box `yaw`

![](/images/2019-09-25-06-01-23.png)

## Euler
The order of operations  
is roll, pitch, yaw around a fixed body frame axis  
(the original frame of the object before rotation is applied).  
Roll is a rotation about x, pitch is about y, yaw is about z.  

```cpp
ignition::math::v4::Quaterniond o = 
pImuSensor->Orientation();
gzmsg << "Roll: " << o.Euler()[0] << "\n";
gzmsg << "Pitch: " << o.Euler()[1] << "\n";
gzmsg << "Yaw: " << o.Euler()[2] << "\n";
```

# Demo2
- Publish euler orientation 

```cpp
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math.hh>
#include <sensors/sensors.hh> // sensor manager
#include <math.h>

namespace gazebo
{
class ModelMove : public ModelPlugin
{
public:
  void Init()
  {
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());
this->lastUpdateTime = this->model->GetWorld()->SimTime();
    std::string poseTopic = std::string("~/") + this->model->GetName() +
                            "/my_euler_pose";
    this->posePub = node->Advertise<gazebo::msgs::Vector3d>(poseTopic);
  }
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->model = _parent;

    this->pid = common::PID(1, 0, 0);
    this->joint = this->model->GetJoint("b2c");
    this->model->GetJointController()->SetVelocityPID(
        this->joint->GetScopedName(), this->pid);
    this->model->GetJointController()->SetVelocityTarget(
        this->joint->GetScopedName(), this->SPEED);

    //init sensor manager
    this->sensor_mgr = sensors::SensorManager::Instance();
    sensors::SensorPtr pSensor = this->sensor_mgr->GetSensor("imu_sensor");
    if (pSensor == nullptr)
    {
      gzerr << "fail get sensor\n";
    }
    this->pImuSensor = std::dynamic_pointer_cast<sensors::ImuSensor, sensors::Sensor>(pSensor);
    if (pImuSensor == nullptr)
    {
      gzerr << "fail load imu data\n";
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelMove::OnUpdate, this));
  }

  void OnUpdate()
  {
    ignition::math::v4::Quaterniond o = pImuSensor->Orientation();
    // gzmsg << o.Yaw() << "," << o.Euler()[2] << "\n";
    gzmsg << o.Roll() << "," << o.Pitch() << "," << o.Yaw() << "\n";
    gazebo::msgs::Vector3d p;
    p.set_x(o.Roll());
    p.set_y(o.Pitch());
    p.set_z(o.Yaw());
    this->posePub->Publish(p);
  }

private:
  transport::PublisherPtr posePub;
  gazebo::transport::NodePtr node;
  sensors::ImuSensorPtr pImuSensor;
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection;
  physics::JointPtr joint;
  common::PID pid;
  sensors::SensorManager *sensor_mgr;
  const int SPEED = 1;
   private: common::Time lastUpdateTime;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelMove)
} // namespace gazebo
```

# Reference
- [Read IMU sensor from a plugin](http://answers.gazebosim.org/question/20276/read-imu-sensor-from-a-plugin/)
- [Ignition math](https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/classignition_1_1math_1_1Quaternion.html)
- [px4 gimbal header](https://github.com/PX4/sitl_gazebo/blob/master/include/gazebo_gimbal_controller_plugin.hh)
- [px4 gimbal cpp](https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_gimbal_controller_plugin.cpp)