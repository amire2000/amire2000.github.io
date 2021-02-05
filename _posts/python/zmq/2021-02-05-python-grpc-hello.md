---
layout: post
title: Python gRPC hello world
categories: python
tags: [protobuf, grpc]
description: Google’s gRPC provides a framework for implementing RPC By layering on top of HTTP/2 and using protocol buffers
image: grpc-icon.png
public: true
---

# Install
```bash
python3 -m venv venv
#activate
pip install --upgrade pip
pip install grpcio

# gRPC tools
pip install grpcio-tools
```

### gRPC tools
Include the protocol buffer compiler protoc and the special plugin for generating server and client code from .proto service definitions

&nbsp;  
&nbsp;  
&nbsp;  
# Hello step by step
- Crate procedure to call by gRPC (hello.py)
- Create proto file declare Messages and services (hello.proto)
- Create Server side (hello_server.py)
- Create Client side (hello_client.py)


```bash
src
├── hello_client.py
├── hello_pb2_grpc.py   #Generated
├── hello_pb2.py        #Generated
├── hello.proto
├── hello.py
└── hello_server.py

```

## define remote procedure (hello.py)

```python
def hello_world():
    return "Hello gRPC"
```

## proto file (hello.proto)
Declare msg and services

- Using protobuf Empty msg
- Define return msg
- Define service `Hello` with one `rpc`
  
```proto
syntax = "proto3";
import "google/protobuf/empty.proto";

message msg {
    string data = 1;
}

service Hello {
    rpc SayHello(google.protobuf.Empty) returns (msg) {}
}
```

### run tool
- generated protobuf and grpc files
    - hello_pb2.py
    - hello_pb2_grpc.py
  
```bash
python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. hello.proto
```

## Server code (hello_server.py)
```python
import signal
import grpc
from concurrent import futures
import time

# import the generated classes
import hello_pb2
import hello_pb2_grpc

# import rpc implement
import hello

class HelloServicer(hello_pb2_grpc.HelloServicer):
    def SayHello(self, request, context):
        response = hello_pb2.msg()
        data = hello.hello_world()
        response.data = data
        return response

def start_server():
    hello_pb2_grpc.add_HelloServicer_to_server(HelloServicer(), server)
    
    print('Starting server. Listening on port 50051.')
    server.add_insecure_port('127.0.0.1:50051')
    server.start()
    

def signal_handler(sig, _):
    exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    start_server()
    print ("Press Ctrl+C to exit.")
    signal.pause()
```

## Client (hello_client.py)
- Create Channel
- Create stub
- Run rpc


```python
import grpc
import hello_pb2
import hello_pb2_grpc
from google.protobuf.empty_pb2 import Empty

channel = grpc.insecure_channel("127.0.0.1:50051")
stub = hello_pb2_grpc.HelloStub(channel)
empty = Empty()

response = stub.SayHello(empty)
print(response.data)
```

