---
layout: post
title: Python zmq and protocol buffer
categories: python
tags: [protobuf, zmq]
description: ZMQ Pub/Sub and protobuf as message infrastructure implement as python and cpp
image: protobuf.png
public: true
---

# LAB
- Implement Pub/Sub with protobuf message
  - Python and CPP
- Send Between Py-Py
- Send Between Py-Cpp
- Send Between Cpp-Py
- Send Between Cpp-Py

# lab project struct
```
├── bin
├── build
├── msgs
│   └── demo.proto
├── proto
├── pub_pb.cpp
├── pub_pb.py
├── sub_pb.cpp
├── sub_pb.py
└── meson.build
```

# pb message

```
syntax = "proto3";
package my_ns;

message Msg{
    string data = 1;
}
```

## compile pb
- Create python and cpp pb messages

```bash
#cpp
protoc --proto_path=msgs --cpp_out=proto msgs/demo.proto
# python
protoc --proto_path=msgs --python_out=proto msgs/demo.proto
```

# Python
## pub
```python
import zmq
import time
from datetime import datetime
from proto.demo_pb2 import Msg
port = "5556"

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:%s" % port)
while True:
    time.sleep(1)
    pb_msg = Msg()
    pb_msg.data = datetime.now().strftime("%HH %M:%S")
    buf = pb_msg.SerializeToString()
    socket.send_multipart([b'topicname', buf])
    print("send: {}".format(pb_msg.data))

```

## sub
```python
import zmq
import time
from proto.demo_pb2 import Msg
port = "5556"

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:%s" % port)
topic_filter = ""
socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)
m = Msg()
while True:
    topic, raw = socket.recv_multipart()
    m.ParseFromString(raw)
    print(m.data)
```

&nbsp;  
&nbsp;  
&nbsp;  
# CPP
## pub
```cpp
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <proto/demo.pb.h>

int main()
{
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://*:5556");
    std::string header("TOPIC");
    std::string data("This is A message");
    while (true)
    {
        my_ns::Msg m;
        m.set_data(data);
        auto buf = m.SerializeAsString();
        zmq::message_t topic((void *)header.c_str(), header.length(), NULL); // 3
        zmq::message_t body((void *)buf.c_str(), buf.length(), NULL);
        publisher.send(topic, ZMQ_SNDMORE);
        publisher.send(body);   
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
```

## sub
```cpp
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <proto/demo.pb.h>

int main()
{
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5556");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, 
        "", 
        0); // 1

    zmq::message_t envelope;
    zmq::message_t content;
    while (true)
    {
        subscriber.recv(&envelope);
        subscriber.recv(&content);
        
        std::string env_msg(
            static_cast<const char*>(envelope.data()), envelope.size());
        std::cout << env_msg << std::endl;

        std::cout << " content: ";
        std::string message(
            static_cast<const char*>(content.data()), content.size());
        my_ns::Msg m;
        m.ParseFromString(message);
        std::cout << m.data() << std::endl;
    }
}
```

## meson build
```python
project('cpp_t', 'cpp',
    version : '0.1',
    default_options : ['warning_level=3', 'cpp_std=c++11'])
zmq = dependency('libzmq')
pb = dependency('protobuf')
install_dir = meson.source_root() + '/bin'

pub_sources = ['pub_pb.cpp', 'proto/demo.pb.cc']
executable('pub_pb', pub_sources,
        dependencies : [zmq, pb],
        install: true,
        install_dir: [install_dir])

sub_sources = ['sub_pb.cpp', 'proto/demo.pb.cc']
executable('sub_pb', sub_sources,
        dependencies : [zmq, pb],
        install: true,
        install_dir: [install_dir])

```
