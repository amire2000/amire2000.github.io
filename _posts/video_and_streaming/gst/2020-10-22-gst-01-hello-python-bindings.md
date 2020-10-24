---
layout: post
title: 01 - Gstreamer Python bindings - pipeline hello
categories: video
tags: [gst, python, python-bindings]
public: true
---

# Install

```bash
python3 -m venv venv
source venv/bin/activate
#
pip install --upgrade wheel pip setuptools

pip install git+https://github.com/jackersson/gstreamer-python.git#egg=gstreamer-python
```

# Launch Pipe from python binding
Run gstreamer pipeline from code has two common method
- parse and launch
- build pipeline
```
gst-launch-1.0 videotestsrc num-buffers=50 ! autovideosink
```

> code example from [LifeStyleTransfer](http://lifestyletransfer.com/how-to-launch-gstreamer-pipeline-in-python/)

## launch
```python
import sys
import traceback
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initializes Gstreamer, it's variables, paths
Gst.init(sys.argv)

PIPELINE = "videotestsrc num-buffers=100 ! autovideosink"

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

pipeline = Gst.parse_launch(PIPELINE)
bus = pipeline.get_bus()
# allow bus to emit messages to main thread
bus.add_signal_watch()
# Start pipeline
pipeline.set_state(Gst.State.PLAYING)
# Init GObject loop to handle Gstreamer Bus Events
loop = GLib.MainLoop()

# Add handler to specific signal
bus.connect("message", on_message, loop)

try:
    loop.run()
except Exception:
    traceback.print_exc()
    loop.quit()

# Stop Pipeline
pipeline.set_state(Gst.State.NULL)
```

&nbsp;  
&nbsp;  
&nbsp;  
## build
```python
import sys
import traceback
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initializes Gstreamer, it's variables, paths
Gst.init(sys.argv)

PIPELINE = "videotestsrc num-buffers=100 ! autovideosink"

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

pipeline = Gst.Pipeline()
src_name = "src"
src = Gst.ElementFactory.make("videotestsrc", src_name)
src.set_property("num-buffers", 50)
src.set_property("pattern", "ball")

sink = Gst.ElementFactory.make("autovideosink")
# add to pipe
pipeline.add(src)
pipeline.add(sink)
# link elements
src.link(sink)

bus = pipeline.get_bus()
# allow bus to emit messages to main thread
bus.add_signal_watch()
# Start pipeline
pipeline.set_state(Gst.State.PLAYING)
# Init GObject loop to handle Gstreamer Bus Events
loop = GLib.MainLoop()

# Add handler to specific signal
bus.connect("message", on_message, loop)

try:
    loop.run()
except Exception:
    traceback.print_exc()
    loop.quit()

# Stop Pipeline
pipeline.set_state(Gst.State.NULL)
```
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [How to install Gstreamer Python Bindings](http://lifestyletransfer.com/how-to-install-gstreamer-python-bindings/)
- [How to launch Gstreamer pipeline in Python](http://lifestyletransfer.com/how-to-launch-gstreamer-pipeline-in-python/)
- [gst-python-tutorials](https://github.com/jackersson/gst-python-tutorials)
- [Gstreamer Symbol mapping C/Python](https://lazka.github.io/pgi-docs/Gst-1.0/mapping.html)





## example
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

{% gist 28ecd4c2fbd10cc17d3e17a02dba9ca1 %}