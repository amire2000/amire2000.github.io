---
layout: post
title: PX4 custom Stream mavlink message
categories: PX4
tags: [px4, mavlink, stream, custom]
---
![](/images/mavlink.png)
# Stream / Publish mavlink custom message
- Create and build uORB msg
- Generated new mavlink message
  - Copy generated folder into PX4 firmware
- PX4 Firmware
  - Declare Stream class in `mavlink_messages.cpp`
  - Register the stream class
- Build and test
  

## uORB message
PX4 uORB messages locate in `Firmware/msg` folder
- Add custom msg file `custom_counter.msg`
- Add reference in CMakeLists.txt
- Run make `make px4_sitl_default`

> uORB messages build into `build/px4_sitl_default/uORB/topics` folder in SITL
  
- custom_counter.msg file
```
uint64 timestamp					# time since system start (microseconds)
uint16 counter
```

- Add to CMakeLists.txt in `Firmware/msg`
```
    set(msg_files
    ...
    custom_counter.msg
	)
```

## Create mavlink message
- Declare message file xml `custom_messages.xml`
- Generated with `mavgenerate.py` (mavlink folder)
- Copy generated c code folder into `PX4` 
  
> mavlink custom folder : `mavlink/message_definitions/v1.0`

### Mavlink message xml file
- Include `common.xml`
  
custom_messages.xml
```xml
<?xml version="1.0"?>
<mavlink>
    <include>common.xml</include>
    <!-- NOTE: If the included file already contains a version tag, remove the version tag here, else uncomment to enable. -->
    <!--<version>3</version>-->
    <enums>
    </enums>
    <messages>
        <message id="166" name="CUSTOM_COUNTER">
            <description>This message just count</description>
            <field type="uint64_t" name="timestamp">Timestamp in milliseconds since system boot</field>
            <field type="uint64_t" name="counter">counter.</field>
            
        </message>
    </messages>
</mavlink>
```

### Generated mavlink
- Language `c`
- Protocol `2.0`
- XML: select custom message xml file
- Out: 
  
![](/images/2019-05-05-18-36-15.png)


### Copy generated into
copy / move `custom_messages` folder into `Firmware/mavlink/include/mavlink/v2.0` folder

![](/images/px4_autopilot.png)
# Firmware custom message support
- mavlink_messages.cpp
  - Add `Stream` class and declaration
  - Register `Stream` publish interval
- mavlink_main.cpp
  - Configure stream ()
  
## mavlink_messages.cpp
- Add Includes
- Add `MavlinkStramXXX` class
- Add to `StreamListItem streams_list[]`

### Add Include
```c
#include <uORB/topics/custom_counter.h>
#include <v2.0/custom_messages/mavlink_msg_custom_counter.h>
```

### Add StreamClass
- Send function check for new ORB message and then send mavlink message down the stream

```cpp
class MavlinkStreamCustomCounter : public MavlinkStream
{
public:
    const char *get_name() const
    {
        return MavlinkStreamCustomCounter::get_name_static();
    }

    static const char *get_name_static()
    {
        return "CUSTOM_COUNTER";
    }

    static uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_CUSTOM_COUNTER;
    }

    uint16_t get_id()
    {
        return get_id_static();
    }

    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamCustomCounter(mavlink);
    }

    unsigned get_size()
    {
        return MAVLINK_MSG_ID_CUSTOM_COUNTER_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    MavlinkOrbSubscription *_sub;
    uint64_t custom_counter_time;

    /* do not allow top copying this class */
    MavlinkStreamCustomCounter(MavlinkStreamCustomCounter &);
    MavlinkStreamCustomCounter& operator = (const MavlinkStreamCustomCounter &);

protected:
    explicit MavlinkStreamCustomCounter(Mavlink *mavlink) : MavlinkStream(mavlink),
        _sub(_mavlink->add_orb_subscription(ORB_ID(custom_counter))),  // make sure you enter the name of your uorb topic here
        custom_counter_time(0)
    {}

    bool send(const hrt_abstime t)
    {
		
        struct custom_counter_s _custom_message;    //make sure ca_trajectory_s is the definition of your uorb topic

        if (_sub->update(&custom_counter_time, &_custom_message)) {

            mavlink_custom_counter_t msg;make sure mavlink_ca_trajectory_t is the definition of your custom mavlink message 

            msg.timestamp = _custom_message.timestamp;
            msg.counter = _custom_message.counter;
			//msg.timestamp = 1;
            //msg.counter = 255;
            

            mavlink_msg_custom_counter_send_struct(
				_mavlink->get_channel(), 
				&msg);
			// PX4_INFO("aaaa");
			return true;
        }
		else{
			return false;
		}
    }
};
```

### Add StreamClass to list
```cpp
StreamListItem(&MavlinkStreamCustomCounter::new_instance, &MavlinkStreamCustomCounter::get_name_static, &MavlinkStreamCustomCounter::get_id_static)
```


## mavlink_main.cpp
### Register Stream class at startup
> TODO: I Place the call in task_main, check if is the right place

```cpp
int
Mavlink::task_main(int argc, char *argv[])
{
...
...
#1.0f => 1 second interval
configure_stream("CUSTOM_COUNTER", 1.0f);
}
```
## Build and Test
- Build
```
make px4_sitl_default
```

- Test
> TODO

# Reference
- [custom mavlink](https://dev.px4.io/en/middleware/mavlink.html)
- [Stream](https://dev.px4.io/en/middleware/modules_communication.html)
# stream
- [Custom MAVLink message for Pixhawk native firmware PX4](https://codar.club/blogs/custom-mavlink-message-for-pixhawk-native-firmware-px4.html)