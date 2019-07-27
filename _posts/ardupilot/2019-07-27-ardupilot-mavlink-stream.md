---
layout: post
title: Ardupilot mavlink stream
categories: APM
tags: [mavlink, stream, dronekit]
image: mavlink_logo.png
description: Set stream rate param and use Dronkit on_message decorate to handle message
public: True
---
# Content
- Stream rate parameters
- Dronkit on_message


# SRX Parameters
SR == Stream Rate
- SR0: usb port
- SR1: Telem 1
- SR2: Telem 2


## Example
Set stream rate in Hertz

![](/images/2019-07-27-13-26-32.png)

- Dronekit register for mavlink stream message
  - for example `SRx_POSITION` control stream rate for `GLOBAL_POSITION_INT` and `LOCAL_POSITION_NED`

```python
from dronekit import connect, VehicleMode
import dronekit_sitl
import time

sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

vehicle = connect(connection_string, wait_ready=True)
last_time = time.time()


@vehicle.on_message('GLOBAL_POSITION_INT')
def listener(self, name, message):
    current = time.time()
    global last_time
    delta = current - last_time
    last_time = current
    print(f"Time delta in ms: {delta}")
    print('message: %s' % message)


# Get/Set Vehicle Parameters
print("\nRead and write parameters")
print(f" Read vehicle param 'SR0_POSITION': {vehicle.parameters['SR0_POSITION']}")

rate_value = 1
print(f" Write vehicle param 'SR0_POSITION' : {rate_value}")
vehicle.parameters['SR0_POSITION'] = rate_value
print(f" Read new value of param 'TSR0_POSITION': {vehicle.parameters['THR_MIN']}")


time.sleep(2)


print("\nClose vehicle object")
vehicle.close()

if sitl is not None:
    sitl.stop()
```


# References
- [Copter SRX parameters](http://ardupilot.org/copter/docs/parameters.html#sr0-parameters)
- [MAVLink Messages](https://dronekit.netlify.com/guide/mavlink_messages.html)
  

# To Read
- [Adding a new MAVLink Message](http://ardupilot.org/dev/docs/code-overview-adding-a-new-mavlink-message.html)
- [How to send a new Mavlink message from Ardupilot?](https://robotics.stackexchange.com/questions/5164/how-to-send-a-new-mavlink-message-from-ardupilot)
- [How to create your own custom messages in MAVlink?](https://discuss.ardupilot.org/t/how-to-create-your-own-custom-messages-in-mavlink/26818)