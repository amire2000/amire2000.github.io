---
layout: post
title: Cross Compiler and Remote Debug
categories: hw
tags: [rpi, cross-compiler, tool-chains, gdb, gdbserver, remote-debug]
public: true
image: toolchain.png
description: Compile c++ for raspberry Pi using gnu cross compiler , using vscode and gdb-multiarch to remote debug
---

LAB
- Install ubuntu os for raspberry pi 
- Install cross compiler
- Write hello app
- Build from cli
- Config VSCode 
    - Cross compiler
    - Remote debugger
    

# Ubuntu for RPi
![](/images/2020-03-03-21-41-32.png)
> 32 bit armhf architecturearmhf
> 
&nbsp;  
&nbsp;  
&nbsp;  
# Toolchain
- Download arm tool chain

```bash
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
# for 32 bit cross
sudo apt-get install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
```
&nbsp;  
&nbsp;  
&nbsp;  
# Hello_world
```cpp
#include <iostream>

using std::cout;

int main(){
    int i = 0;
    cout << "Hello Rpi\n";
    return 0;
}
```
&nbsp;  
&nbsp;  
&nbsp;  
# Build 
- cli

```bash
# /usr/bin
aarch64-linux-gnu-g++ -g main.cpp
```

# VSCode Config
## launch
- program location
- debugger to use `gdb-multiarch` (`sudo apt install gdb-multiarch`)
- remote ip
  
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "GDB remote debug",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceRoot}/src/a.out",
            "args": [],
            "stopAtEntry": true,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "targetArchitecture": "arm64",
            // "preLaunchTask": "prepdebug",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ],
            "miDebuggerPath": "/usr/bin/gdb-multiarch",
            "miDebuggerServerAddress": "192.168.2.249:3000"
        }
    ]
}
```
&nbsp;  
&nbsp;  
&nbsp;  
# Pi side
- Upload binary using scp or other method
- Install gdbserver (`sudo apt install gdbserver`)
- Run `gdbserver`

```
gdbserver :3000 a.out
```

> Tip: Using static gdbserver
> https://github.com/hugsy/gdb-static

&nbsp;  
&nbsp;  
&nbsp;  
# VSCode config cont.
- Task
- Deploy
- CMake

## Task
- Run Cross compiler
- Deploy

### Cross compiler
- Build using CrossCompiler
  
```json
{
    "label": "cross compiler",
    "type": "shell",
    "command": "/usr/bin/aarch64-linux-gnu-g++",
    "args": [
        "-g",
        "${file}",
        "-o",
        "${workspaceFolder}/bin/${fileBasenameNoExtension}"
    ]
}
```

## Deploy
### using scp

```
scp bin/main ubuntu@<ip>:app
```

### Using Extension

![](/images/2020-03-13-08-01-43.png)
- [Extension wiki](https://github.com/mkloubert/vscode-deploy-reloaded/wiki/target_sftp)

- Deploy our cross binary
  - Set target
    - defined command after `upload`
  - Set Package
    - Defined button on status as deploy shortcut
  
```json
{
    "deploy.reloaded": {
        "targets": [
            {
                "name": "rpi",
                "type": "sftp",
                "host": "<ip>",
                "user": "<user>",
                "password": "<pass>",
                "dir": "<path>",
                "commands": {
                    "uploaded": [
                        "chmod 777 ${remote_file}"
                    ]
                },
            }
        ],
        "packages": [
            {
                "name": "cross_demo",
                 "button": {
                    "text": "Deploy RPi",
                    "tooltip": "Upload files ...",

                    "targets": [ "rpi" ]
                },
                "files": [
                    "/bin/main"
                ]
            }
        ]
    }
}
```
&nbsp;  
&nbsp;  
&nbsp;  
# CMake
Basic `CMakeLists.txt` using cross compiler 

```cmake
cmake_minimum_required(VERSION 3.10)
project(hello_cross)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 11)

set(CROSS_PATH /usr/bin)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CXX_COMPILER ${CROSS_PATH}/aarch64-linux-gnu-g++)

add_executable(main src/main.cpp)
```


&nbsp;  
&nbsp;  
&nbsp;  
# References
  - [Cross architecture remote debugging using gdb with Visual Studio Code (vscode) on Linux](https://medium.com/@karel.l.vermeiren/cross-architecture-remote-debugging-using-gdb-with-visual-studio-code-vscode-on-linux-c0572794b4ef)