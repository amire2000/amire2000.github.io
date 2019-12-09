---
layout: post
title: ZMQ Pub/Sub multipart message
categories: cpp
tags: [zmq, python, pub_sub, multipart]
public: true
description: Using zmq pub/sub to send message with topic / envelope, show how to usages with cpp and python
---

# Examples
- c++
  - multipart cpp pub
  - multipart cpp sub
  - meson build
- python
  - multipart python pub
  - multipart python sub

# multipart cpp pub

```cpp
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
int main()
{
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://*:5556");
    std::string aEnv("A");
    std::string aCon("This is A message");
    std::string bEnv("B");
    std::string bCon("This is B message");

    while (true)
    {
        zmq::message_t ma1((void *)aEnv.c_str(), aEnv.length(), NULL); // 3
        zmq::message_t ma2((void *)aCon.c_str(), aCon.length(), NULL);
        publisher.send(ma1, ZMQ_SNDMORE);
        publisher.send(ma2);             

        zmq::message_t mb1((void *)bEnv.c_str(), bEnv.length(), NULL);
        zmq::message_t mb2((void *)bCon.c_str(), bCon.length(), NULL);
        publisher.send(mb1, ZMQ_SNDMORE);
        publisher.send(mb2);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
```

# multipart cpp pub

```cpp
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
int main()
{
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5556");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, 
        "", 
        0); // 1

    while (true)
    {
        zmq::message_t envelope;
        zmq::message_t content;
        subscriber.recv(&envelope);
        subscriber.recv(&content);

        std::cout << "Envelope: ";
        std::string env_msg(
            static_cast<const char*>(envelope.data()), envelope.size());
        std::cout << env_msg << std::endl;

        std::cout << " content: ";
        std::string message(
            static_cast<const char*>(content.data()), content.size());
        std::cout << message << std::endl;
        
    }
}
```

# meson build

```python
project('cpp_t', 'cpp',
    version : '0.1',
    default_options : ['warning_level=3', 'cpp_std=c++11'])
zmq = dependency('libzmq')

install_dir = meson.source_root() + '/bin'

pub_sources = ['multipub.cpp']
executable('multi_pb', pub_sources,
        dependencies : [zmq],
        install: true,
        install_dir: [install_dir])

sub_sources = ['multisub.cpp']
executable('multi_sub', sub_sources,
        dependencies : [zmq],
        install: true,
        install_dir: [install_dir])
```

# multipart python pub

```python
import zmq
import time
context = zmq.Context()
pub = context.socket(zmq.PUB)
pub.bind("tcp://*:5556")
while True:
    time.sleep(1)
    msg = "This is A message from python"
    pub.send_multipart([b'A', msg.encode('utf-8')])
```

# multipart python sub

```python
import zmq

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect ("tcp://localhost:5556")
topicfilter = ""
socket.setsockopt_string(zmq.SUBSCRIBE, topicfilter) # pylint: disable=maybe-no-member

while True:
    topic, msg = socket.recv_multipart()
    print(topic, msg)
```