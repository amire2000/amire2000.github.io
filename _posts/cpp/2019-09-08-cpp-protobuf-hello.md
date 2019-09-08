---
layout: post
title: Google protobuf cpp hello world
categories: cpp
tags: [protobuf]
image:
public: true
description: Google protobuf hello world in cpp
---

```cpp
#include <iostream>
#include "messages.pb.h"

using namespace std;
int main()
{
    Messages::Msg m;
    m.set_data("hello proto");
    auto buf = m.SerializeAsString();
    Messages::Msg new_m;
    new_m.ParseFromString(buf);
    cout << new_m.data() << endl;
    return 0;
}
```

## meson
```python
protoc = find_program('protoc')
dep = dependency('protobuf')

gen = generator(protoc, \
  output    : ['@BASENAME@.pb.cc', '@BASENAME@.pb.h'],
  arguments : ['--proto_path=@CURRENT_SOURCE_DIR@/proto/', 
  '--cpp_out=@BUILD_DIR@', '@INPUT@'])
# @BUILD_DIR@
generated = gen.process('proto/messages.proto')
e = executable('prog', 'main.cpp', generated,
dependencies : dep)
```