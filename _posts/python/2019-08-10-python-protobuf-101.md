---
layout: post
title: Python Protocol buffer 101
categories: python
tags: [protobuf]
description: First step usage protobuf, install python support and config vscode 
image: protobuf.png
public: true
---

# Protocol buffer
Protocol Buffers is a way to serialize structured data into a binary stream in a fast and efficient manner. It is designed to be used for inter-machine communication and remote procedure calls (RPC)


# Content
- [Install](#install)
- [Hello protocol buffer](#Protobuf-hello-world)
- [Message](#message)
  - [Array](#array)
  - [Nested](#nestedimport-message)
  - [Enums](#enums)
  - [Packages and Import]
- [API](#api)
- [Tips](#Tips)

&nbsp;  
&nbsp;  
&nbsp;  
# install
- Download and install protobuf compiler
- Install python libraries
  
## Install protoc compiler 
Download the correct binary for example [protoc-3.9.1-linux-x86_64.zip](https://github.com/protocolbuffers/protobuf/releases)

## Install in virtualenv
```bash
(venv)$ pip install protobuf
```
&nbsp;  
&nbsp;  
# Protobuf hello world

project structure

```
├── src
│   ├── demo.py
│   └─ proto
│          ├── __init__.py
│          ├── msg_pb2.py
│          └── msg.proto
├──.env   
└── venv
```


## Defining Protobuf Messages
- Define messages
- Compile messages with `protoc` compiler
- Usage

### Define message
```
syntax = "proto3";

message DemoMsg {
    int32 id = 1;
    string name = 2;
}
```
### Compile
> Don't forget to compile using `protoc`
> `protoc` can be usage from 
>   - command line  
>   - defined `task`
>   - vscode-proto3 ext

#### Command line example 
  
`protoc -I=$SRC_DIR --python_out=$DST_DIR $SRC_DIR/*.proto`

&nbsp;  
&nbsp;  
## usage
```python
from proto import msg_pb2

p = msg_pb2.DemoMsg()
p.id = 1
p.name = "demo 1"
buf = p.SerializeToString()
pp = msg_pb2.DemoMsg()
pp.ParseFromString(buf)
print (pp)
```
&nbsp;  
&nbsp;  
&nbsp;  
# Message
## Array
### proto  msg
```
syntax = "proto3";

message Foo {
    repeated int32 items = 1;
}
```

### code
```python
from proto import msg_pb2
from google.protobuf import json_format

m = msg_pb2.Foo()
m.items.append(1)
m.items.extend([2, 3])
d = json_format.MessageToDict(m)
print (d)
```

### Result
```
{'items': [1, 2, 3]}
```
&nbsp;  
&nbsp;  
## Nested/Import message

### msg
> Same result if we declare the nested message outside the parent message (class)
```
syntax = "proto3";

message DemoMsg {
    int32 id = 1;
    string name = 2;
    repeated Foo foo = 3;
}

message Foo {
    string phone = 1;
    string address = 2;
}
```

### code
```python
from proto import msg_pb2
from google.protobuf import json_format

m = msg_pb2.DemoMsg()
m.id = 1
m.name = "nested"
n = m.foo.add()
n.phone = "12345"
n.address = "public"

n = m.foo.add()
n.phone = "00000"
n.address = "private"

d = json_format.MessageToJson(m)
print (d)
```

### output
```json
{
  "id": 1,
  "name": "nested",
  "foo": [
    {
      "phone": "12345",
      "address": "public"
    },
    {
      "phone": "00000",
      "address": "private"
    }
  ]
}
```

&nbsp;  
&nbsp;  
&nbsp;  
## Enums
Enums are expanded by the metaclass into a set of symbolic constants with integer values

### proto msg
```
```

### code
```python
from proto import msg_pb2
from google.protobuf import json_format

m = msg_pb2.Phone()
m.phone_number = "12345"
m.type = msg_pb2.Phone.phone_type.WORK
print(json_format.MessageToJson(m))
```

### result
```
{
  "phoneNumber": "12345",
  "type": "WORK"
}
```
&nbsp;  
&nbsp;  
&nbsp;  
# API

```python
from proto import msg_pb2
from google.protobuf import json_format

p = msg_pb2.DemoMsg()
p.id = 1
p.name = "demo 1"

json = json_format.MessageToJson(p)
print (type(json))
print (json)
print("---")
message = json_format.Parse(json, msg_pb2.DemoMsg())
print (type(message))
print (message)
print("---")
json = json_format.MessageToDict(p)
print (type(json))
print (message)
```

### out
```bash
<class 'str'>
{
  "id": 1,
  "name": "demo 1"
}
---
<class 'msg_pb2.DemoMsg'>
id: 1
name: "demo 1"

---
<class 'dict'>
id: 1
name: "demo 1"
```
&nbsp;  
&nbsp;  
&nbsp;  
# Tips
## PYTHONPATH
- Setup `PYTHONPATH` using .env file
    -  place `.env` file at the project root folder
  
```
PYTHONPATH=./src:${PYTHONPATH}
```

## protoc task
```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "protoc",
            "type": "shell",
            "command": "protoc",
            "args": [
                "-I",
                "${workspaceFolder}/src/pb_demo/proto",
                "--python_out",
                "${workspaceFolder}/src/pb_demo/proto",
                "${workspaceFolder}/src/pb_demo/proto/*.proto"
            ]
        }
    ]
}
```
&nbsp;  
&nbsp;  
&nbsp;  
## Publisher
```python
PRICE_TOPIC = b'price'

def make_price_update_bytes():
    price_update = price_update_pb2.PriceUpdate()
    price_update.timestamp = str(utcdatetime.utcdatetime.now())
    price_bytes = price_update.SerializeToString()
    return price_bytes

def send_messages(socket):
    logging.debug("Sending messages to topics.")
    price_bytes = make_price_update_bytes()
    socket.send(PRICE_TOPIC + b' ' + price_bytes)

def main():
    logging.basicConfig(level=logging.DEBUG)
    context = zmq.Context()
    socket = context.socket(zmq.PUB)

    socket.bind("tcp://*:{}".format(PORT))
    try:
        while True:
            send_messages(socket)
            time.sleep(5)

```

## Subscriber
```python
def worker1():
    context = zmq.Context()

    socket = context.socket(zmq.SUB)

    socket.connect("tcp://{ip}:{port}".format(ip=PUBLISHER_IP, port=PORT))
    socket.setsockopt(zmq.SUBSCRIBE, TOPIC)

    while True:
        topic_and_data = socket.recv()
        msg = topic_and_data.split(b' ', 1)
        log.info(msg[0])
        data = msg[1]
        log.info(repr(data) + '\n')

def main():
    t1 = Thread(target=worker1)
    t1.setDaemon(True)
    t1.start()

    t1.join()
```

# Reference
- [Protobuf Language Guide](https://developers.google.com/protocol-buffers/docs/proto3)
- [Exploring Google Protobuffers with Python](https://dev.to/chen/exploring-google-protobuffers-with-python-1gmd)
- [Using pyZMQ for inter-process communication: Part 1](https://www.pythonforthelab.com/blog/using-pyzmq-for-inter-process-communication-part-1/)
- [Using pyZMQ for inter-process communication: Part 2](https://www.pythonforthelab.com/blog/using-pyzmq-for-inter-process-communication-part-2/)