---
layout: post
title: 04 - Gstreamer Python bindings - cairo hello
categories: video
tags: [gst, python, cairo, gstreamer, python-bindings]
public: true
description: Draw on video using cairo
---

# Install 
```
```

# check
```bash
gst-inspect-1.0 | grep cairo
cairo:  cairooverlay: Cairo overlay

# inspect
gst-inspect-1.0 cairooverlay

Element Signals:
  "draw" :  void user_function (GstElement* object,
                                CairoContext* arg0,
                                guint64 arg1,
                                guint64 arg2,
                                gpointer user_data);
  "caps-changed" :  void user_function (GstElement* object,
                                        GstCaps* arg0,
                                        gpointer user_data);

```

# Hello cairo
Cairo is a 2D graphics library with support for multiple output devices.

# Demo code
```python
#!/usr/bin/env python
import cairo
import gi
import sys
import traceback
gi.require_version('Gst', '1.0')
gi.require_foreign('cairo')
from gi.repository import Gst, GLib


def on_draw(overlay, context, timestamp, duration):
    """Each time the 'draw' signal is emitted"""
    context.select_font_face('Open Sans', cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL) # pylint: disable=maybe-no-member
    context.set_font_size(40)
    context.move_to(100, 100)
    context.text_path('HELLO')
    context.set_source_rgb(0.5, 0.5, 1)
    context.fill_preserve()
    context.set_source_rgb(0, 0, 0)
    context.set_line_width(1)
    context.stroke()

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

Gst.init(sys.argv)
PIPELINE = "videotestsrc num-buffers=100 ! cairooverlay name=overlay ! videoconvert ! autovideosink"

pipeline = Gst.parse_launch(PIPELINE)
cairo_overlay = pipeline.get_by_name('overlay')
cairo_overlay.connect('draw', on_draw)

bus = pipeline.get_bus()
bus.add_signal_watch()
pipeline.set_state(Gst.State.PLAYING)
loop = GLib.MainLoop()
bus.connect("message", on_message, loop)

try:
    loop.run()
except Exception:
    traceback.print_exc()
    loop.quit()

# Stop Pipeline
pipeline.set_state(Gst.State.NULL)
```