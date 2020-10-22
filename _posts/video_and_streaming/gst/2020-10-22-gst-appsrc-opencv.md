---
layout: post
title: Push OpenCV frame into appsrc gstreamer pipe
categories: video
tags: [gst, appsrc, push-buffer, python-bindings]
public: true
description: Push OpenCV frame into appsrc gstreamer pipe
---

```python
# Base on : https://github.com/tamaggo/gstreamer-examples/blob/master/test_gst_appsrc_testvideo_mp4mux.py
# GStreamer-CRITICAL **: 19:07:14.276: gst_segment_to_stream_time: assertion 'segment->format == format' failed
# Solved by add appsrc name=src is-live=true format=time (format) to pipe command
import cv2
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst

I420 = "I420"
RGB = "RGB"
PIX_TYPE = RGB

class SampleGenerator:
    def __init__(self):
        self.t = 0
        self._last_t_v = 0
        self.fps = 10.0

    def gen(self, source):
        if self.t - self._last_t_v >= 1.0/self.fps:
            data = np.zeros((240, 320, 3), dtype=np.uint8)
            fontFace = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            thickness = 1
            color = (0, 255, 255)
            text = "%6f" % self.t
            oh = 0#v[0][1]*2
            v = cv2.getTextSize(text, fontFace, fontScale, thickness)
            cl = int(round(160 - v[0][0]/2))
            cb = int(round(120 + oh - v[1] - v[0][1]/2))
            cv2.putText(data,
            text,
            (cl, cb),
            fontFace, fontScale, color, thickness)
            if I420 in PIX_TYPE:
                data = cv2.cvtColor(data, cv2.COLOR_RGB2YUV)
                y = data[...,0]
                u = data[...,1]
                v = data[...,2]
                u2 = cv2.resize(u, (0,0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
                v2 = cv2.resize(v, (0,0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
                data = y.tostring() + u2.tostring() + v2.tostring()
            else:
                data = data.tostring()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            assert buf is not None
            buf.fill(0, data)
            # Todo: understand
            buf.pts = buf.dts = int(self.t * 1e9)
            source.emit("push-buffer", buf)
            self._last_t_v = self.t
        self.t += 0.000001
        

if __name__ == "__main__":
    GObject.threads_init()
    Gst.init(None)
    pipe = "appsrc name=src is-live=true format=time ! videoconvert !  autovideosink"

    pipeline = Gst.parse_launch(pipe)
    pipeline.set_state(Gst.State.READY)
    pipeline.set_state(Gst.State.PLAYING)

    src = pipeline.get_by_name('src')
    if RGB in PIX_TYPE:
        caps = Gst.Caps.from_string("video/x-raw,format=(string)RGB,width=320,height=240,framerate=30/1")
    elif I420 in PIX_TYPE:
        caps = Gst.Caps.from_string("video/x-raw,format=(string)I420,width=320,height=240,framerate=30/1")

    
    src.set_property("caps", caps)
    src.set_property("format", Gst.Format.TIME)
    g = SampleGenerator()
    for t in np.arange(0, 5, 0.000001):
    # while True:
        g.gen(src)

    src.emit("end-of-stream")
    bus = pipeline.get_bus()
    while True:
        msg = bus.poll(Gst.MessageType.ANY, Gst.CLOCK_TIME_NONE)
        t = msg.type
        if t == Gst.MessageType.EOS:
            print("EOS")
            break
    pipeline.set_state(Gst.State.NULL)

```