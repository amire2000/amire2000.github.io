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
 void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->model = _parent;

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
    else
    {
      /*Do something ... for example to read the linear accel from the IMU:*/
      ignition::math::v4::Quaterniond o = pImuSensor->Orientation();
      o.Yaw();
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelMove::OnUpdate, this));
}

 void OnUpdate()
  {
      ignition::math::v4::Quaterniond o = pImuSensor->Orientation();
      gzmsg << o.Yaw() << "\n";
    }
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

# Run and usage
```bash
ninja -C build install
```