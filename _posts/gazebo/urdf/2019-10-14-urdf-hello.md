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

# URDF
- Gazebo convert URDF to SDF

## check
```bash
sudo apt-get install liburdfdom-tools
#
# check_urdf <urdf file>
# for example
check_urdf basic.urdf
```

## urdf root
```xml
<?xml version="1.0" ?>
<robot name="mobilerobot">
</robot>
```

## link
- visual
- collision
- inertial


### inertia
> Gazebo work in SI units (mass->kg length->m moment of inertia-> Kg*m^2)

### Calc demo
- [Online](https://amesweb.info/inertia/moment-of-inertia-of-rectangular-plate.aspx)

![](/images/2020-05-10-06-18-22.png)

```xml
<link>
...
<inertial>
			<mass value="10"/>
			<inertia 
        ixx="0.833"
        ixy="0.0"
        ixz="0.0"
        iyy="0.833"
        iyz="0.0"
        izz="1.667"/>
</link>
```

#### Origin
- Set Center of mass
- Check inertail origin

From Gazebo menu check
- Wirefream
- Center of mass
- Inertias


```xml
gz model --spawn-file=basic.urdf --model-name=basic 
<inertial>
    <origin xyz="0.0 0.0 -0.02"/>
    <mass value="0.3"/>
    <inertia ixx="0.000433" ixy="0.0" ixz="0.0" iyy="0.00025" iyz="0.0" izz="0.000672"/>
</inertial>

gz model --spawn-file=basic.urdf --model-name=basic -y 0.2
<inertial>
    <mass value="0.3"/>
    <inertia ixx="0.000433" ixy="0.0" ixz="0.0" iyy="0.00025" iyz="0.0" izz="0.000672"/>
</inertial>

gz model --spawn-file=basic.urdf --model-name=basic -y 0.4
<inertial>
    <origin xyz="0.0 0.0 0.02"/>
    <mass value="0.3"/>
    <inertia ixx="0.000433" ixy="0.0" ixz="0.0" iyy="0.00025" iyz="0.0" izz="0.000672"/>
</inertial>




```

![](/images/2020-05-10-23-19-09.png)

## Demo
```
gz model --spawn-file=basic.urdf --model-name=basic
```
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
- [Design URDF](https://gbiggs.github.io/rosjp_urdf_tutorial_text/mobile_robot_urdf.html#%E3%82%B8%E3%83%A7%E3%82%A4%E3%83%B3%E3%83%88%E3%81%AE%E5%AE%9A%E7%BE%A9)
- [Basic example of ROS URDF Pan / Tilt system](https://pinkwink.kr/1007)
- [URDF in Gazebo](http://gazebosim.org/tutorials?tut=ros_urdf)
- [pygazebo](https://pypi.org/project/pygazebo/)