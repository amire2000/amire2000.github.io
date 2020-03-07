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
- Rpi 3+
- Ubuntu 18.04 64bit(not raspbian)
- VSCode

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