---
layout: post
title: VSCode and Docker as dev environment
categories: vscode
tags: [docker, remote]
description: Using docker as as build and debug environment, control action for vscode
image: docker.png
public: true
---
# Objective

Used Docker as build environment

- Create Docker file
- Config VSCode to used docker as build and debug env.

&nbsp;  
&nbsp;  
&nbsp;  
# Project
```
.
├── .vscode
│   ├── launch.json
│   └── tasks.json
├── CMakeLists.txt
├── docker
│   ├── docker-compose.yml
│   ├── Dockerfile
│   └── README.md
└── src
    ├── CMakeLists.txt
    └── main.cpp

```
&nbsp;  
&nbsp;  
&nbsp;  
# Docker
- build minimal env for build and debug
  - Install: compilers and debuggers
  - Add user
  - Expose ports for remote debugger and other services
&nbsp;  
&nbsp;    
## Dockerfile

```dockerfile
FROM ubuntu:18.04
MAINTAINER robobe@dev.com

RUN apt-get update
RUN apt-get install -y \
    sudo \
    build-essential \
    cmake \
    gdb \
    gdbserver \
    openssh-server

RUN useradd -m user && \
echo "user:user" | chpasswd &\
& adduser user sudo

# Clean up APT when done.
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
# for gdbserver
EXPOSE 2000

USER user
CMD /bin/bash
```
&nbsp;    
&nbsp;    
&nbsp;    
# docker compose
- Used by VSCode to bring `up` the build environment include
    - Volumes
    - Ports
  
## docker-compose.yml
```yaml
version: '2'
services:
  dev:
    build: .
    ports:
      - "3322:22"
      - "2000:2000"
    privileged: true
    container_name: dev_env
    volumes: 
      - "../:/home/user/project/"
    working_dir: /home/user/project
```
&nbsp;    
&nbsp;    
&nbsp;    
# VSCode
## tasks.json
- Start/Stop env
- Config/Build
- Run/Debug

> Tip: used docker `-w` option to set working folder


```json
"tasks": [
    {
        "label": "start develop env",
        "type": "shell",
        "command": "docker-compose -f docker/docker-compose.yml run --service-ports --name dev_env dev bash",
        "problemMatcher": []
    },
    {
        "label": "stop develop env",
        "type": "shell",
        "command": "docker-compose -f docker/docker-compose.yml down",
        "problemMatcher": []
    },
    {
        "label": "CMake (Remote)",
        "type": "shell",
        "command": "docker exec -it -w /home/user/project/build dev_env cmake ..",
        "problemMatcher": []
    },
    {
        "label": "Build (make)",
        "type": "shell",
        "command": "docker exec -it -w /home/user/project/build dev_env sh -c 'make -j $(nproc)'",
        "problemMatcher": []
    },
    {
        "label": "Run (remote)",
        "type": "shell",
        "command": "docker exec -it -w /home/user/project/build dev_env ./src/hello",
        "problemMatcher": []
    },
    {
        "label": "GDB Server",
        "type": "shell",
        "command": "docker exec -it -w /home/user/project/build dev_env gdbserver :2000 ./src/hello",
        "problemMatcher": []
    }
]
```
&nbsp;  
&nbsp;  
&nbsp;  
## Debug
- Run Remote debug

> Used `sourceFileMap` to map between Local and Remote folders


```json
"sourceFileMap": {
    <remote path>: <local path>
},
```

### launch.json
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug (Remote)",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceRoot}/build/src/hello",
            "sourceFileMap": {
                "/home/user/project/": "/home/user/projects/cpp_docker/"
            },
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ],
            "miDebuggerPath": "/usr/bin/gdb",
            "miDebuggerServerAddress": "127.0.0.1:2000"
        }
    ]
}
```
&nbsp;  
&nbsp;  
&nbsp;  
## Code example
### main.cpp
```cpp
#include<iostream>

using std::cout;

int main(int argc, char const *argv[])
{
    int i=0;
    cout << " hello \n" ;
    return 0;
}
```

### CMakelists.txt (main)
```cmake
cmake_minimum_required(VERSION 3.10)
project (hello)

set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(src)
```

### CMakelists.txt (src)
```cmake
add_executable(hello main.cpp)
```