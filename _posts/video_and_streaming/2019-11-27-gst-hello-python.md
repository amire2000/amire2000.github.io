---
layout: post
title: Gstreamer python binding
categories: video
tags: [gst, gstreamer]
public: true
---

```
ln -s /usr/lib/python2.7/dist-packages/glib
ln -s /usr/lib/python2.7/dist-packages/gobject
ln -s /usr/lib/python2.7/dist-packages/gst-0.10
ln -s /usr/lib/python2.7/dist-packages/gstoption.so
ln -s /usr/lib/python2.7/dist-packages/gtk-2.0
ln -s /usr/lib/python2.7/dist-packages/pygst.pth
ln -s /usr/lib/python2.7/dist-packages/pygst.py
ln -s /usr/lib/python2.7/dist-packages/pygtk.pth
ln -s /usr/lib/python2.7/dist-packages/pygtk.py
```

```
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject
import threading
import signal

class MainPipeline():
    def __init__(self):
        self.pipeline = None
        self.videosrc = None
        self.videosink = None

    

    def gst_thread(self):
        print("Initializing GST Elements")
        Gst.init(None)

        self.pipeline = Gst.Pipeline.new("hello")

        # instantiate the camera source
        self.videosrc = Gst.ElementFactory.make("videotestsrc", "test")
        self.videosrc.set_property("pattern", "ball")

        # instantiate the appsink - allows access to raw frame data
        self.videosink = Gst.ElementFactory.make("autovideosink", "vid-sink")
        
        # add all the new elements to the pipeline
        print("Adding Elements to Pipeline")
        self.pipeline.add(self.videosrc)
        self.pipeline.add(self.videosink)

        self.videosrc.link(self.videosink)
        # link the elements in order, adding a filter to ensure correct size and framerate
        # print("Linking GST Elements")
        # self.videosrc.link_filtered(self.videoparse,
        #     Gst.caps_from_string('image/jpeg,width=640,height=480,framerate=30/1'))
        # self.videoparse.link(self.videosink)

        # start the video
        print("Setting Pipeline State")
        self.pipeline.set_state(Gst.State.PAUSED)
        self.pipeline.set_state(Gst.State.PLAYING)

def signal_handler(signum, frame):
    print("Interrupt caught")

if __name__ == "__main__":
    pipeline = MainPipeline()
    gst_thread = threading.Thread(target=pipeline.gst_thread)
    gst_thread.start()

    print("registering sigint")
    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()
```