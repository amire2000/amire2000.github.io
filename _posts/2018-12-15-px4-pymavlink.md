---
layout: post
title: PX4 pymavlink
categories: px4
tags: [sitl, gazebo, px4, pymavlink]
---

## Mavlink
MAVLink is a very lightweight messaging protocol for communicating with drones (and between onboard drone components).
- Data stream are send as a topic
- Sub-protocols configuration are point-to-point with retransmission

### Messages
Messages are defined within XML files. Each XML file defines the message set supported by a particular MAVLink system, also referred to as a "dialect".

With `mavgen/macgenerate` tool we convert the `xml` to specfic programing language library to use

### Protocol
MAVLink is a binary protocol
mavlink has 2 major version
- V1.0
- V2.0

Telemtry data are send in a multicast 
Configuration and require guaranteed send in point-to-point with retransmission

### Serialization
The over-the-wire format of MAVLink is optimized for resource-constrained systems and hence the field order is not the same as in the XML specification
[more](https://mavlink.io/en/about/overview.html#serialization)

### Message definition
MAVLink messages are defined in XML files in the mavlink/message definitions folder. The messages that are common to all systems are defined in common.xml (only messages contained in this file are considered standard messages).

## Pymavlink
Pymavlink is a low level and general purpose MAVLink message processing library

> There number of source to get the library
> - From mavgen ()
> - Own project (Ardupilot github)
> - pip (download ardupilot version )

## Reference
- [MAVLink Tutorials](http://cs460.coins-lab.org/index.php?title=MAVLink_Tutorials)
- [PX4 MAVLink messaging](https://dev.px4.io/en/middleware/mavlink.html)
- [Mavlink tutorial for absolute dummies](http://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf)
## Todo 
- install apm v2
- 