---
layout: post
title: Google protobuf cpp hello world
categories: cpp
tags: [protobuf]
image: protobuf.png
public: true
description: Google protobuf hello world in cpp
---

# basic demo
- create message
- compile using `protoc`
- implement basic serialization/de serialization
- compile with meson build


## basic message
- include package / namespace
  
```
syntax = "proto3";

package my_ns;

message Msg {
    string data = 1; 
}
```

## Compile pb message
- from `source`folder
- Create cpp files in the same folder as the message


```bash
protoc --proto_path=proto --cpp_out=proto proto/*.proto
```

## basic implementation
```cpp
#include <iostream>
#include <proto/message.pb.h>

using namespace std;
int main()
{
    my_ns::Msg m;
    m.set_data("hello proto");
    auto buf = m.SerializeAsString();
    my_ns::Msg new_m;
    new_m.ParseFromString(buf);
    cout << new_m.data() << endl;
    return 0;
}
```

## compile with meson
> Add `xxx.pb.cc` file to sources file list  
> [C++ Protobuf undefined reference to constructor/destructor](https://stackoverflow.com/questions/39261897/c-protobuf-undefined-reference-to-constructor-destructor)

```python
dep = dependency('protobuf')
install_dir = meson.source_root() + '/bin'

sources = ['main.cpp', 'proto/message.pb.cc']

e = executable('pb_demo', 
  sources, 
  dependencies : dep,
  install: true,
  install_dir: [install_dir])
```