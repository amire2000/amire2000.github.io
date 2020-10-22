---
layout: post
title: Gstreamer python binding
categories: video
tags: [gst, gstreamer, python-bindings]
public: true
description: Using python gstreeamer binding to create and changed pipelines
---

# Elements
## pipeline
![](/images/2020-09-25-11-46-02.png)

## pads
link element to outside world
pad can add and remove (create and destroy) dynamically from element

- Sink (input) pad
- Source (output) pad

![](/images/2020-09-25-11-56-00.png)

## bus
Take care of internal messaging in GStreamer

### queries
The purpose of query is to get some information from pipeline synchronous, for example ask for bitrate

### events
The purpose if event is to tell the pipeline to take some action in given condition, for example error, EOS

![](/images/2020-09-25-18-40-59.png)

## Plugins
- Protocol handling
- Sources: data input
- Formats: parsers, formaters, muxers, demuxers, metadata
- Codecs
- Filters:
- Sinks:
   
# Diagrams and Graphs

## dot file for pipeline
```
```

## dot file for gst binding
```python
#!/usr/bin/env python3

import cv2
import gi
import traceback

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

pipe = "videotestsrc name=src num-buffers=100 ! video/x-raw,width=640,height=480 ! autovideosink name=sink"
pipeline = Gst.parse_launch(pipe)
# Add for debug / dot file
Gst.debug_bin_to_dot_file(pipeline, Gst.DebugGraphDetails.ALL, "debug_pipe")
pipeline.set_state(Gst.State.READY)
pipeline.set_state(Gst.State.PLAYING)

loop = GLib.MainLoop()
loop.run()
```

```bash
mkdir /tmp/gst
cd /tmp/gst
# run python gst example
GST_DEBUG_DUMP_DOT_DIR=. python3 /home/user/projects/py_tutorial/gst/gst_step1.py
# convert to png
dot -Tpng debug_pipe.dot > p.png
# view
eog p.png
```


# Reference
- [python api](https://lazka.github.io/pgi-docs/Gst-1.0/index.html)
- [Symbol Mapping](https://lazka.github.io/pgi-docs/Gst-1.0/mapping.html)
- [gstreamer pipe line graph](https://developer.ridgerun.com/wiki/index.php/How_to_generate_a_Gstreamer_pipeline_diagram_(graph))
- [python-gst-tutorial](https://github.com/gkralik/python-gst-tutorial)
- [lifestyletransfer](http://lifestyletransfer.com)
# to-read
- [GStreamer Dynamic Pipelines](https://coaxion.net/blog/2014/01/gstreamer-dynamic-pipelines/)



```python
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject
import threading
import signal
import time
import cherrypy

class HelloWorld(object):
    @cherrypy.expose
    def index(self, i=0):
        obj = pipeline.videosrc
        if int(i) == 1:
            obj.set_property("pattern", "ball")
        else:
            obj.set_property("pattern", "snow")

def change_pattern():
    time.sleep(1)
    obj = pipeline.videosrc
    obj.set_property("pattern", "ball")

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
        self.videosrc.set_property("pattern", "snow")

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

    # gst_thread = threading.Thread(target=change_pattern)
    # gst_thread.setDaemon(True)
    # gst_thread.start()
    cherrypy.quickstart(HelloWorld())

    print("registering sigint")
    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()

```