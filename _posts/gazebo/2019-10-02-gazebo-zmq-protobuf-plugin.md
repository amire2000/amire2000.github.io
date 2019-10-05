---
layout: post
title: Send message from python to gazebo using ZMQ and Protobuf
categories: gazebo
tags: [zmq, protobuf]
description: Integrated ZMQ with protobuf in gazebo model plugin
public: true
---


# Source code
- CPP
{% gist bc20c603e3f5d9257b4a701923c45794 %}

- HEADER

```cpp
#ifndef SOCKET_COM_H
#define SOCKET_COM_H

// Boost imports
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

//Gazebo imports
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>

// ZeroMq and Protobuf imports
#include <zmq.hpp>
// #include "zhelpers.hpp"
// #include "agv_base.pb.h"

namespace gazebo{

class SocketCom : public ModelPlugin
{
public:
    /// \brief constructor
    SocketCom();

    /// \brief destructor
    virtual ~SocketCom();

    /// \brief Plugin Load function
    /// \param[in] model pointer to the model defining this plugin
    /// \param[in] sdf pointer to the SDF of the model
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate();

private:
   /// \brief Subscribe(): Receiving Data from Endpoinrt
   void Subscribe(void);

   /// \brief Publish(): Sending Data to Endpoint
   void Publish(void);

   static void signal_handler(int _signal);

private:
    //Gazebo
    physics::ModelPtr model;               /**< Holds the model state */
    physics::WorldPtr world;               /**< Holds the world state */
    sdf::ElementPtr sdf;                   /**< Holds the model definition */
    event::ConnectionPtr _updateConnection;
    static int s_instance_cnt;             /**< Instance Counter */
    int agv_id;                            /**< Holds AGV ID Number */

    //ZMQ
    std::string endpoint;                  /**< Endpoint */
    std::string protocol;                  /**< Protocol Type TCP/INPROC..*/
    std::string ip_adress;                 /**< IP of Socket Endpoint */
    int port_start_number;                 /**< Start Port Socket Number */

    boost::shared_ptr<zmq::context_t> context;          /**< ZMQ Context Instance, io_service by default one thread */
    boost::shared_ptr<zmq::socket_t> zmq_subscriber;    /**< ZMQ Socket Subscriber Instance */
    boost::shared_ptr<zmq::context_t> context_pub;          /**< ZMQ Context Instance, io_service by default one thread */
    boost::shared_ptr<zmq::socket_t> zmq_publisher;    /**< ZMQ Socket Subscriber Instance */

    //Thread
    boost::thread thread_sub;
    boost::thread thread_pub;
    std::mutex signal_mutex;

    static std::atomic<bool> s_terminate;

    /// \brief Pointer to the update event connection.
    ///        Handles the controller loop, called every phsics step.
    event::ConnectionPtr update_events;

    //Transport
    transport::NodePtr node;                /**< Transport node */
    transport::SubscriberPtr gz_subscriber; /**< Subscriber that listens to the agv sensor fdbk message */
    transport::PublisherPtr gz_publisher;   /**< Publisher that sends the current agv state into the gazebo env. */
};
}//namespace

#endif // SOCKET_COM_H
```

- Meson compile
```python
gz = dependency('gazebo')
zmq = dependency('libzmq')

install_dir = meson.source_root() + '/bin'

sources = ['zmq_plugin/socket_com.cpp']
myplug = library('my_zmq', sources: sources,
        dependencies : [gz, zmq],
        install: true,
        install_dir: [install_dir])
```

- World tester
```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">

    <gui>
      <plugin name="sample" filename="libgui_example_spawn_widget.so"/>
    </gui>

    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="mysphere">
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
       
        <collision name="collision">
        <geometry>
            <sphere radius="0.01"/>
        </geometry>
        </collision>
        <visual name="visual">
          <geometry>
              <sphere radius="0.01"/>
            
          </geometry>
          <material>
            <lighting>1</lighting>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
        <inertial>
        <pose frame=''>0 0 0 0 0 0</pose>
        <mass>10</mass>
        <inertia>
          <ixx>0.16</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.16</iyy>
          <iyz>0</iyz>
          <izz>0.16</izz>
        </inertia>
      </inertial>
      </link>

      <plugin name="zmq_test" filename="libmy_zmq.so"/>
    </model>        
  </world>
</sdf>
```

- python subscriber
```python
import sys
import zmq

NO_FILTER = ""

context = zmq.Context()
socket = context.socket(zmq.SUB)

socket.connect ("tcp://localhost:5564")

topicfilter = NO_FILTER
socket.setsockopt_string(zmq.SUBSCRIBE, topicfilter)

while (True):
    string = socket.recv()
    print (string)
```
&nbsp;  
&nbsp;  
# Reference
- [How to communicate with GZServer over a custom Socket-Client](http://answers.gazebosim.org/question/13321/how-to-communicate-with-gzserver-over-a-custom-socket-client/)