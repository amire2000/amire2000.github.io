---
layout: post
title: Gazebo URDF and XACRO hello
categories: gazebo
tags: [urdf, xacro]
public: true
description: URDF and XACRO hello world, How to spawn and using urdf/xacro in gazebo, check urdf tree and embedded gazebo sensor and other tags.
image: urdf.png
---
The Universal Robotic Description Format (URDF) is an XML file format used in ROS to describe all elements of a robot.

# urdf and gazebo
[http://gazebosim.org](http://gazebosim.org/tutorials?tut=ros_urdf)   
To use a URDF file in Gazebo, some additional simulation-specific tags must be added to work properly with Gazebo

- inertia: 
An `<inertia>` element within each `<link>` element must be properly specified and configured.

- gazebo (robot)
  - plugin
    ```xml
    <robot>
        ...
        <gazebo>
            <plugin name="model_plugin" filename="libmy_joint_vel.so"/>
        </gazebo>
    </robot>
    ```
- gazebo (link)
  - Visual color
    ```xml
    <link name="link1">
        ...
    </link>
    <gazebo reference="link1">
        <material>Gazebo/Orange</material>
    </gazebo>
    ```

  - Sensors
    ```xml
    <link name="link2">
        ...
    </link>
    <gazebo reference="link2">
		<sensor name="imu_sensor" type="imu">
			<pose>0 0 0 0 0 0</pose>
			<always_on>1</always_on>
			<update_rate>10.0</update_rate>
		</sensor>
	</gazebo>
    ```

# urdf
## link
![](/images/2019-10-15-08-29-08.png)

## joint
![](/images/2019-10-15-08-29-36.png)

```xml
<robot name="test_robot">
	<link name="link1">
		<collision>
			<origin xyz="0 0 0.05" rpy="0 0 0"/>
			<geometry>
				<box size="0.1 0.1 0.1"/>
			</geometry>
		</collision>
		<visual>
			<origin xyz="0 0 0.05" rpy="0 0 0"/>
			<geometry>
				<box size="0.1 0.1 0.1"/>
			</geometry>
			<material name="orange"/>
		</visual>
		<inertial>
			<origin xyz="0 0 1" rpy="0 0 0"/>
			<mass value="1"/>
			<inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
		</inertial>
	</link>
	<gazebo reference="link1">
		<material>Gazebo/Orange</material>
	</gazebo>
	<link name="link2">
		<collision>
			<origin xyz="0 0 0.06" rpy="0 0 0"/>
			<geometry>
				<box size="0.1 0.1 0.1"/>
			</geometry>
		</collision>
		<visual>
			<origin xyz="0 0 0.06" rpy="0 0 0"/>
			<geometry>
				<box size="0.1 0.1 0.1"/>
			</geometry>
			<material name="green"/>
		</visual>
		<inertial>
			<origin xyz="0 0 1" rpy="0 0 0"/>
			<mass value="1"/>
			<inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
		</inertial>
	</link>
	<gazebo reference="link2">
		<material>Gazebo/Green</material>
	</gazebo>
	<joint name="b2c" type="continuous">
		<parent link="link1"/>
		<child link="link2"/>
		<origin xyz="0 0 0.1" rpy="0 0 0"/>
		<axis xyz="0 0 1"/>
	</joint>
	<gazebo>
		<plugin name="model_plugin" filename="libmy_joint_vel.so"/>
	</gazebo>
	<gazebo reference="link2">
		<sensor name="imu_sensor" type="imu">
			<pose>0 0 0 0 0 0</pose>
			<always_on>1</always_on>
			<update_rate>10.0</update_rate>
		</sensor>
	</gazebo>
</robot>
```

# Join plugin
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
- [Basic example of ROS URDF Pan / Tilt system](https://pinkwink.kr/1007)
- [URDF in Gazebo](http://gazebosim.org/tutorials?tut=ros_urdf)
- [pygazebo](https://pypi.org/project/pygazebo/)