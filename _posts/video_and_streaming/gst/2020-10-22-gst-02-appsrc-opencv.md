---
layout: post
title: 02 - Gstreamer Python bindings - Push OpenCV into appsrc
categories: video
tags: [gst, appsrc, push-buffer, python-bindings]
public: true
description: Push OpenCV frame into appsrc gstreamer pipe
---


# Demo
```python

import sys
import os
import time
import traceback
import cv2
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst

class SampleGenerator:
    def __init__(self, src):
        self._last_t_v = -31337
        self._last_t_a = -31337
        self._last_t_s = -31337
        self._src = src

    def gen(self, t):
        if t - self._last_t_v >= 1.0/30:
            data = np.zeros((240, 320, 3), dtype=np.uint8)
            data = cv2.cvtColor(data, cv2.COLOR_RGB2YUV)

            fontFace = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            thickness = 1
            color = (0, 255, 255)
            text = "%6f" % t
            oh = 0  # v[0][1]*2
            v = cv2.getTextSize(text, fontFace, fontScale, thickness)
            cl = int(round(160 - v[0][0]/2))
            cb = int(round(120 + oh - v[1] - v[0][1]/2))
            cv2.putText(data, text, (cl, cb), fontFace,
                        fontScale, color, thickness)

            y = data[..., 0]
            u = data[..., 1]
            v = data[..., 2]
            u2 = cv2.resize(u, (0, 0), fx=0.5, fy=0.5,
                            interpolation=cv2.INTER_AREA)
            v2 = cv2.resize(v, (0, 0), fx=0.5, fy=0.5,
                            interpolation=cv2.INTER_AREA)
            data = y.tobytes() + u2.tobytes() + v2.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            assert buf is not None
            buf.fill(0, data)
            buf.pts = buf.dts = int(t * 1e9)
            self._src.emit("push-buffer", buf)
            self._last_t_v = t


def on_message(bus: Gst.Bus, message: Gst.Message, loop: GLib.MainLoop):
    mtype = message.type
    if mtype == Gst.MessageType.EOS:
        print("End of stream")
        loop.quit()

    elif mtype == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(err, debug)
        loop.quit()

    elif mtype == Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        print(err, debug)

    return True


Gst.init(None)
my_src = Gst.ElementFactory.make("appsrc", "vidsrc")
my_convert = Gst.ElementFactory.make("videoconvert", "vidcvt")
my_sink = Gst.ElementFactory.make("autovideosink", "my_sink")
pipeline = Gst.Pipeline()
pipeline.add(my_src)
pipeline.add(my_convert)
pipeline.add(my_sink)
caps = Gst.Caps.from_string("video/x-raw,format=(string)I420,width=320,height=240,framerate=30/1")
my_src.set_property("caps", caps)
my_src.set_property("format", Gst.Format.TIME)

my_src.link(my_convert)
my_convert.link(my_sink)

pipeline.set_state(Gst.State.PLAYING)

bus = pipeline.get_bus()
# allow bus to emit messages to main thread
bus.add_signal_watch()
# Start pipeline
pipeline.set_state(Gst.State.PLAYING)
# Init GObject loop to handle Gstreamer Bus Events
loop = GLib.MainLoop()

# Add handler to specific signal
bus.connect("message", on_message, loop)
s = SampleGenerator(my_src)
for t in np.arange(0, 5, 0.000001):
    s.gen(t)

my_src.emit("end-of-stream")


try:
    loop.run()
except Exception:
    traceback.print_exc()
    loop.quit()

# Stop Pipeline
pipeline.set_state(Gst.State.NULL)

```