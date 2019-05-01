---
layout: post
title: px4 opticflow
categories: px4
tags: [px4, opticflow]
---

## Optical flow
Optical Flow uses a downward facing camera and a downward facing distance sensor for position estimation

LPE : Local Position Estimator
EKF2:  Extended Kalman Filter, fusion sensor output to the filter

## Optical flow gazebo
- <px4>/sitl_gazebo/models/iris_opt_flow
  - iris
  - px4flow
    - imu
    - camera
      - libgazebo_opticalflow_plugin.so

### gazebo_opticalflow_plugin
- source: `Firmware/Tools/sitl_gazebo/src`

- width: 64
- height: 64
- fov: 0.25 rad
- `focal_length_ = (this->width/2)/tan(hfov_/2);`
- publish `opticalFlow_message`
  - x
  - y
  - imu
    - x,y,z

-  gazebo  `gazebo_mavlink_interface` subscribe to optic message
   -  publish `mavlink_hil_optical_flow_t` mavlink msg with opticflow sensor data

- Mavlink
  - #114: HIL_OPTICAL_FLOW
  - #106: OPTICAL_FLOW_RAD
  - 