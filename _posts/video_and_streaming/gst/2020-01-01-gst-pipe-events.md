---
layout: post
title: Gstreamer pipe events
categories: video
tags: [gst, events]
public: true
---

```python
#!/usr/bin/env python3

import cv2
import gi
import traceback

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

def on_new_buffer(sink):
    try:
        # A datatype to hold a time, measured in nanoseconds.
        print(sink.clock.get_internal_time())
    except Exception as e:
        print('ERROR', traceback.format_exc())

    return Gst.FlowReturn.OK


GObject.threads_init()
Gst.init(None)

pipe = "videotestsrc name=src ! video/x-raw,width=640,height=480 ! appsink name=sink"
pipeline = Gst.parse_launch(pipe)

sink = pipeline.get_by_name('sink')
sink.set_property("max-buffers",2)
sink.set_property("emit-signals", True)
sink.connect("new-sample", on_new_buffer)

pipeline.set_state(Gst.State.READY)
pipeline.set_state(Gst.State.PLAYING)

loop = GObject.MainLoop()
loop.run()



```