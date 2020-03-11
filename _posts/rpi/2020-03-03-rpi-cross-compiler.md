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
- HW: Rpi 3+
- Ubuntu 18.04 64bit(not raspbian)
  - gdbserver
- VSCode
- gdb-multiarch

![](/images/2020-03-03-21-41-32.png)
> 32 bit armhf architecturearmhf
> 
# Toolchain
- Download arm toolchain

```bash
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
# for 32 bit cross
sudo apt-get install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
```

# Hello_world
```cpp
```

## build from cli
```
aarch64-linux-gnu-g++ -g main.cpp

```

# VSCode remote debug
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

# Pi side
- Run `gdbserver`

```
gdbserver :3000 a.out
```

# References
  - [Cross architecture remote debugging using gdb with Visual Studio Code (vscode) on Linux](https://medium.com/@karel.l.vermeiren/cross-architecture-remote-debugging-using-gdb-with-visual-studio-code-vscode-on-linux-c0572794b4ef)