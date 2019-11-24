---
layout: post
title: gstreamer first step
categories: video
tags: [streaming, h264]
public: True
description: Install and basic usage, create pipes, encode and stream over network
image: gstreramer.png
---
## Install
```bash
sudo apt-get install gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-doc \
  gstreamer1.0-tools
```
## Basic
Gstreamer difine a pipeline, The pipe contain elements the first  element are source  and the last are sink
 
![](images/d7d59066.png)

The  basic pipe contain source and a  sink

```bash
gst-launch-1.0 videotestsrc ! autovideosink
```

## Elements / Plugins
### Input elements
- v4l2src
- fdsrc
- videotestsrc
- udpsrc

### Sink elements
- filesink
- autovideosink
- udpsink
- multiudpsink

## caps
Caps / Capabilities describe the type off data that is streamed between two pads / elements
- **caps do not modify data**
- They ensure compatibility between elements

## Other type of element
- tee
- queue
- Converter
  - color change
  - cropping
  - videoscale
  - rotation
  - videoconvert
- Payers/Depayers: prepare data (payload) to network transport 
  - rtp
    - rtph264/rtph264depay

## Tools
- gst-inspect: discover gstreamer  elements
- gst-launch: build a  gstreamer pipeline on the command line

### bash TAB auto complete
- [gst-launch tab-completion](http://gstreamer-devel.966125.n4.nabble.com/gst-launch-tab-completion-td4657538.html)
```bash
# wget https://raw.github.com/drothlis/gstreamer/bash-completion-master/tools/gstreamer-completion
cp gstreamer-completion  /etc/bash_completion.d/
source /etc/bash_completion.d/gstreamer-completion
```

&nbsp;  
&nbsp;  
&nbsp;  
# Network stream
##  Streaming  pipeline
### Sender
- acquire video data
- compress
- cut the data into smaller data
- send over the network

###  Reciver
- recive packets from  the network
- reassemble
- decompress the video
- display

## Demo
Gstreamer  pipline  that use h264  as codec and rtp as transmit protocol over udp
- `videotestsrc`  and `autovideosink` to  genereate and display an image
- `x264enc` and `` as codec
  
```bash
#sender
gst-launch-1.0 videotestsrc \
! video/x-raw,width=640,height=480 \
!  jpegenc \
! rtpjpegpay \
! udpsink host=127.0.0.1 port=1234

#receiver
gst-launch-1.0 -v udpsrc port=1234 \
! application/x-rtp, encoding-name=JPEG,payload=26 \
! rtpjpegdepay \
! jpegdec \
! autovideosink


#sender
gst-launch-1.0 videotestsrc \
! video/x-raw,width=640,height=480 \
! x264enc  tune=zerolatency byte-stream=true \
 bitrate=3000 threads=2 \
! rtph264pay \
! udpsink host=127.0.0.1 port=1234

#receiver
gst-launch-1.0 -v udpsrc port=1234 \
! application/x-rtp, encoding-name=H264,payload=96 \
! rtph264depay \
! avdec_h264 ! \
 autovideosink


```

&nbsp;  
&nbsp;  
&nbsp; 
### Video
- install `sudo  apt install v4l-utils`
- My camera support to types of video capture
```bash
v4l2-ctl --list-formats -d /dev/video1
ioctl: VIDIOC_ENUM_FMT
	Index       : 0
	Type        : Video Capture
	Pixel Format: 'YUYV'
	Name        : YUYV 4:2:2

	Index       : 1
	Type        : Video Capture
	Pixel Format: 'MJPG' (compressed)
	Name        : Motion-JPEG

```
- For example in 1280*720 format type: YUYV its support only  tow frame rate
```bash
v4l2-ctl --list-formats-ext -d /dev/video1
....
Size: Discrete 1280x720
			Interval: Discrete 0.100s (10.000 fps)
			Interval: Discrete 0.133s (7.500 fps)
....

#fail to work framerate to  high 
gst-launch-1.0 v4l2src device="/dev/video1" ! video/x-raw,width=1280,height=720,framerate=30/1 ! videoconvert ! autovideosink
# framerate supoorted
gst-launch-1.0 v4l2src device="/dev/video1" ! video/x-raw,width=1280,height=720,framerate=10/1 ! videoconvert ! autovideosink
```

- Convert video to gray
```bash
gst-launch-1.0 v4l2src device="/dev/video1" ! \
    video/x-raw,width=640,height=480 ! \
    videoconvert ! \
    video/x-raw,format=GRAY8 ! \
    videoconvert ! \
    autovideosink
```
### Networking
```bash
#sender
gst-launch-1.0 -v v4l2src device=/dev/video1 ! video/x-raw,width=640,height=480 !  jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=1234

#receiver
gst-launch-1.0 -v udpsrc port=1234 ! application/x-rtp, encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink

```

## Python support
### Install
```bash
sudo apt-get install python-gst-1.0 python3-gst-1.0
```

### Example
```python
#!/usr/bin/env python

import os
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, Gtk

class GTK_Main(object):

    def __init__(self):
        window = Gtk.Window(Gtk.WindowType.TOPLEVEL)
        window.set_title("control panel")
        window.set_default_size(500, 100)
        window.connect("destroy", Gtk.main_quit, "WM destroy")
        vbox = Gtk.VBox()
        window.add(vbox)
        hbox = Gtk.HBox()
        vbox.pack_start(hbox, False, False, 0)

        self.button = Gtk.Button("480")
        hbox.pack_start(self.button, False, False, 0)
        self.button.connect("clicked", self.start_stop)

        self.button_format = Gtk.Button("color")
        hbox.pack_start(self.button_format, False, False, 0)
        self.button_format .connect("clicked", self.changed_format)

        self.movie_window = Gtk.DrawingArea()
        vbox.add(self.movie_window)
        window.show_all()

        # gst - launch - 1.0
        # v4l2src
        # device = "/dev/video1" ! \
        #         video / x - raw, width = 640, height = 480 ! \
        #         videoconvert ! \
        #         video / x - raw, format = GRAY8 ! \
        #         videoconvert ! \
        #         autovideosink
        self.player = Gst.Pipeline.new("my pipe")
        source = Gst.ElementFactory.make("v4l2src", "file-source")
        source.set_property("device", "/dev/video1")

        filter = Gst.ElementFactory.make("capsfilter", "filter")
        caps = Gst.Caps.from_string("video/x-raw, width=160, height=120")
        filter.set_property("caps", caps)

        textoverlay = Gst.ElementFactory.make('textoverlay')
        textoverlay.set_property("text", "GNUTV")
        textoverlay.set_property("font-desc", "normal 14")

        convert = Gst.ElementFactory.make("videoconvert", "convert")

        filter_color_space = Gst.ElementFactory.make("capsfilter", "filter_color")
        caps = Gst.Caps.from_string("video/x-raw, format=GRAY8")
        filter_color_space.set_property("caps", caps)

        convert_out = Gst.ElementFactory.make("videoconvert", "convert_out")

        sink = Gst.ElementFactory.make("autovideosink", "video-output")

        self.player.add(source)
        self.player.add(filter)
        self.player.add(textoverlay)
        self.player.add(convert)
        self.player.add(filter_color_space)
        self.player.add(convert_out)
        self.player.add(sink)

        source.link(filter)
        filter.link(textoverlay)
        textoverlay.link(convert)
        convert.link(filter_color_space)
        filter_color_space.link(convert_out)
        convert_out.link(sink)

        self.player.set_state(Gst.State.PLAYING)

    def changed_format(self, w):
        if self.button.get_label() == "color":
            title = "gray"
            caps = Gst.Caps.from_string("video/x-raw, format=RGB")
        else:
            title = "color"
            caps = Gst.Caps.from_string("video/x-raw, format=GRAY8")

        self.button_format.set_label(title)
        self.player.get_by_name("filter_color").set_property("caps", caps)
        a=self.player.get_by_name("filter_color").get_property("caps")
        print a

    def start_stop(self, w):
        # 160x120
        # 640*480
        if self.button.get_label() == "480":
            self.button.set_label("160")
            caps = Gst.Caps.from_string("video/x-raw, width=640, height=480,framerate=10/1")
        else:
            self.button.set_label("480")
            caps = Gst.Caps.from_string("video/x-raw, width=160, height=120")

        self.player.get_by_name("filter").set_property("caps", caps)
        self.player.set_state(Gst.State.NULL)
        self.player.set_state(Gst.State.PLAYING)

Gst.init(None)
GTK_Main()
GObject.threads_init()
Gtk.main()
```

##  Reference
- [Gstreamer cheat sheet](http://wiki.oz9aec.net/index.php/Gstreamer_cheat_sheet#Network_Streaming)
- [GStreamer-1.0 personal cheat sheet](https://gist.github.com/strezh/9114204)
- [Play webcam using gstreamer](https://medium.com/@petehouston/play-webcam-using-gstreamer-9b7596e4e181)
- [Videostreaming with Gstreamer](http://z25.org/static/_rd_/videostreaming_intro_plab/index.html)
- [Wide Area Network Emulator](https://sourceforge.net/projects/wanem/)
- [How to launch Gstreamer pipeline in python](http://lifestyletransfer.com/how-to-launch-gstreamer-pipeline-in-python/)
- [gstreamer python example git](https://github.com/brettviren/pygst-tutorial-org)
- [Python GStreamer Tutorial](http://brettviren.github.io/pygst-tutorial-org/pygst-tutorial.html)
- [getting to know gstreamer](http://westside-consulting.blogspot.com/2017/03/getting-to-know-gstreamer.html)
- [Mission planner gstreamer](https://discuss.ardupilot.org/t/mission-planner-gstreamer-pipeline-question/35254/2)