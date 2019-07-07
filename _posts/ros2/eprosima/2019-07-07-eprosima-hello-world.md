---
layout: post
title: eProsima hello world
categories: ros2
tags: [rtps, dds]
image: eprosima.png
public: true
description: eprosima Fast RTPS is a C++ implementation of the RTPS (Real Time Publish-Subscribe) protocol, which provides publisher-subscriber communications over unreliable transports such as UDP
---
eprosima Fast RTPS is a C++ implementation of the RTPS (Real Time Publish-Subscribe) protocol, which provides publisher-subscriber communications over unreliable transports such as UDP.  
eProsima Fast RTPS has been adopted by multiple organizations in many sectors including these important cases:
- Robotics: ROS2 default middleware
- FIWARE: A market-ready open source software, combining components that enable the connection to IoT with Context Information Management and Big Data services in the Cloud

# Content
- 3 words acronyms
  - Fast RTPS
  - Fast CDR
  - DDS
  - IDL
- Installation (linux)
- Simple Hello World

# 3 words acronyms
## Fast RTPS
eProsima Fast RTPS is a high performance publish subscribe framework to share data in distributed systems using a decoupled model based on Publishers, Subscribers and Data Topics.

## Fast CDR
eProsima FastCDR library provides two serialization mechanisms. One is the standard CDR serialization mechanism, while the other is a faster implementation of it. 

## DDS
Data Distribution Service for real time system is a OMG specification for a pub/sub data centric model.

## IDL
OMG Interface Description Language

&nbsp;  
&nbsp;  
# Installation
- Clone from git
- Create build folders


```
git clone https://github.com/eProsima/Fast-RTPS
mkdir Fast-RTPS/build && cd Fast-RTPS/build

cd eProsima_FastCDR-1.0.8-Linux 
./configure --libdir=/usr/lib 
make -j2 
sudo make install

cd eProsima_FastRTPS-1.7.1-Linux 
./configure --libdir=/usr/lib 
make -j2 
sudo make install
```

## Install folders
- include:
    - /usr/local/include/fastcdr/
- lib
    - /usr/local/lib
- cmake
    -  /usr/local/share/fastcdr/cmake/


# Simple Hello World
- Decalre `idl` file
- Generated with `fastrtpsgen`
- Changed generated file for more clarity
- Run

## Project structure

```
hello_eprosima
  └── build
  └── src
```

## idl message

- MyStruct.idl
```
struct MyType
{
    long value;
    string message;
};
```
 
## Generated

```bash
# output generated files into build folder
#from project src folder
fastrtpsgen -example cmake -d ../build MyStruct.idl
```
### Changed publish and subscriber
#### Publisher
- MyTypePublisher.cxx
  - Add code in `void MyTypePublisher::run()` method
  
```cpp
MyType st;

/* Initialize your structure here */
st.value = 1;
st.mess = "Hello world";
```

#### Subscriber
- MyTypeSubscriber.cxx
  - Add code in `void MyTypeSubscriber::SubListener::onNewDataMessage(Subscriber* sub)` method to print more info about the incoming message

```cpp
// Print your structure data here.
++n_msg;
std::cout << "Sample received, count=" << n_msg 
<< "Message data: " << st.mess << std::endl;
```

&nbsp;  
&nbsp;  
&nbsp;  
## Compile
> Important 
> `export fastcdr_DIR=/usr/local/share/fastcdr/cmake/`

```bash
cd build
cmake ..
make
```

&nbsp;  
&nbsp;  
&nbsp;  

## Run
> From `build` folder


- Terminal 1 (subscriber)

```bash
./MyType subscriber
#
Starting 
Waiting for Data, press Enter to stop the Subscriber. 
Subscriber matched
Sample received, count=1
```

- Terminal 2 (publisher)

```bash
./MyType publisher
#
Starting 
Publisher created, waiting for Subscribers.
Publisher matched
Sending sample, count=1, send another sample?(y-yes,n-stop):
```

# Reference
- [eProsima fast RTPS](https://eprosima-fast-rtps.readthedocs.io/en/1.3.1/introduction.html)
- [eProsima Fast RTPS: PubSub Hello World](https://www.youtube.com/watch?v=JW9yWhekpW4)