---
layout: post
title: 03 - Gstreamer Python bindings - Pull gstreamer buffer into opencv
categories: video
tags: [gst, appsink, push-buffer, python-bindings]
public: true
description: Pull frame from appsink gstreamer pipe
---

# Demo code
```python
import cv2
import numpy as np
from gi.repository import Gst, GLib
import sys
import traceback
import gi
gi.require_version('Gst', '1.0')

Gst.init(sys.argv)
image_arr = None

PIPELINE = "videotestsrc num-buffers=100 \
    ! video/x-raw,format=BGR,width=640,height=480 \
    ! appsink name=sink"


def gst2np(sample: Gst.Sample) -> np.ndarray:
    buffer = sample.get_buffer()
    caps = sample.get_caps()
    # print(buffer.pts, buffer.dts, buffer.offset)
    format = caps.get_structure(0).get_value('format')
    height = caps.get_structure(0).get_value('height')
    width = caps.get_structure(0).get_value('width')
    buffer_size = buffer.get_size()
    array = np.ndarray(
        shape=(height, width, 3), 
        buffer=buffer.extract_dup(0, buffer_size),
        dtype=np.uint8)
    return np.squeeze(array)

def on_new_buffer(sink, data):
    sample = sink.emit("pull-sample")  # Gst.Sample
    if isinstance(sample, Gst.Sample):
        array = gst2np(sample)
        global image_arr
        image_arr = array
        return Gst.FlowReturn.OK
    return Gst.FlowReturn.ERROR


pipeline = Gst.parse_launch(PIPELINE)

sink = pipeline.get_by_name('sink')
sink.set_property("emit-signals", True)
sink.connect("new-sample", on_new_buffer, None)

bus = pipeline.get_bus()
bus.add_signal_watch()
pipeline.set_state(Gst.State.PLAYING)
loop = GLib.MainLoop()

while True:
    message = bus.timed_pop_filtered(10000, Gst.MessageType.ANY)
    
    if message:
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(("Error received from element %s: %s" % (
                message.src.get_name(), err)))
            print(("Debugging information: %s" % debug))
            break
        elif message.type == Gst.MessageType.EOS:
            print("End-Of-Stream reached.")
            break
        elif message.type == Gst.MessageType.STATE_CHANGED:
            if isinstance(message.src, Gst.Pipeline):
                old_state, new_state, pending_state = message.parse_state_changed()
                print(("Pipeline state changed from %s to %s." %
                       (old_state.value_nick, new_state.value_nick)))
        else:
            print("Unexpected message received.")
            print(message.type)

    if image_arr is not None:
        cv2.imshow("appsink image arr", image_arr)
        cv2.waitKey(1)

pipeline.set_state(Gst.State.NULL)
```