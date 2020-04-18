---
layout: post
title: C++ ZMQ Request-Replay
categories: cpp
tags: [zmq]
description:
public: true
image: zeromq.jpg
---
# Request-Replay
The REQ-REP socket pair work with sequence, client send and wait for recv, server wait in recv and send response.

## Server example
- Wait for message (`recv`)
- Send replay message (`send`)


## Build
- cli
  
```
g++ client.cpp -o client -lzmq
```

- cmake

```cmake 
find_package(cppzmq)

add_executable(server server.cpp)
target_link_libraries(server cppzmq)

add_executable(client client.cpp)
target_link_libraries(client cppzmq)
```

&nbsp;  
&nbsp;  
&nbsp;  
## multiple socket
- [Multiple socket poller in C++](http://zguide.zeromq.org/cpp:mspoller)
&nbsp;  
&nbsp;  
&nbsp;  
# Reference

- [ZMQ Guide](http://zguide.zeromq.org/page:all)