---
layout: post
title: ardupilot send text from FCU to GCS
categories: apm
tags: [debug]
---
`GCS_MAVLink::send_text()` A simple way to send small short debug messages (50 characters or less) to the ground station

# Usage example
<!-- - Add `#include <GCS_MAVLink/GCS.h>` to `GCS_MAVLink/GCS.h` file -->

```cpp
#pragma once
// #include <GCS_MAVLink/GCS.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "GCS_MAVLink.h"

...
```
- use `send_text` method
```cpp
const uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        
        gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)3.142f);
        
        gcs().send_message(MSG_HEARTBEAT);
        gcs().send_message(MSG_SYS_STATUS);
    }
```

- View messages at QGC
  
![](/images/2019-06-12-22-56-20.png)

![](/images/2019-06-12-22-42-45.png)

# Reference
- [Debugging with send_text](http://ardupilot.org/dev/docs/debug-with-send-text.html)