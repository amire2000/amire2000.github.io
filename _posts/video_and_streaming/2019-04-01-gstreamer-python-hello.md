---
layout: post
title: gstreamer python first step
categories: gstreamer
tags: [gstreamer, python]
---

## Install python packages
```
sudo apt-get install python-gst-1.0 python3-gst-1.0
```

##  Import
> require version ,ust be before import gst
```
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject
```

## Create a Pipeline
- Parse command
- Element Factory

### Demo pipe
```
gst-launch-1.0 videotestsrc num-buffers=50 ! autovideosink
```

### Parse command
```python
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject
import sys
import traceback

Gst.init(sys.argv)
command = "videotestsrc ! autovideosink"
pipeline = Gst.parse_launch(command)
pipeline.set_state(Gst.State.PLAYING)
loop = GObject.MainLoop()
 
try:
    loop.run()
except:
    traceback.print_exc()
```

### Element Factory
```python
import traceback
import sys
from gi.repository import Gst, GObject
import gi
gi.require_version('Gst', '1.0')

Gst.init(sys.argv)
pipeline = Gst.Pipeline()
src = Gst.ElementFactory.make("videotestsrc")
src.set_property("num_buffers", 50)

sink = Gst.ElementFactory.make("autovideosink")

pipeline.add(src, sink)

src.link(sink)

pipeline.set_state(Gst.State.PLAYING)
loop = GObject.MainLoop()

try:
    loop.run()
except:
    traceback.print_exc()
```
# Reference
- [
How to launch Gstreamer pipeline in Python](http://lifestyletransfer.com/how-to-launch-gstreamer-pipeline-in-python/)
-  [Gst API](https://lazka.github.io/pgi-docs/#Gst-1.0)