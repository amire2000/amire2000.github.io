---
layout: post
title: Gstreamer nvidia
categories: video
tags: [nvidia]
public: true
image: 
description: Streaming video using nvidia encoder 
---

## Reference
- [NVIDIA Jetson Nano GStreamer streaming pipelines](https://developer.ridgerun.com/wiki/index.php?title=Jetson_Nano/Gstreamer/Example_Pipelines/Streaming#OMX_Sender)

# h264
```
gst-launch-1.0 videotestsrc \
! 'video/x-raw, width=640, height=480, framerate=20/1' \
! omxh264enc control-rate=2 bitrate=400000 \
! video/x-h264, stream-format=byte-stream \
! rtph264pay mtu=1400 \
! udpsink host=192.168.2.108 port=5000 sync=false async=false
```

```
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp,encoding-name=H264,payload=96 \
! rtph264depay \
! h264parse \
! queue \
! avdec_h264 \
! xvimagesink sync=false async=false -e
```

# h265
```
gst-launch-1.0 videotestsrc \
! 'video/x-raw, width=640, height=480, framerate=20/1' \
! omxh265enc control-rate=2 bitrate=8000000 \
! video/x-h265, stream-format=byte-stream \
! rtph265pay mtu=1400 \
! udpsink host=192.168.2.108 port=5000 sync=false async=false
```

```
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp,encoding-name=H265,payload=96 \
! rtph265depay \
! h265parse \
! queue \
! avdec_h265 \
! xvimagesink sync=false async=false -e
```

# VP8
```
gst-launch-1.0 videotestsrc \
! 'video/x-raw, width=640, height=480, framerate=20/1' \
! omxvp8enc \
! rtpvp8pay mtu=1400 \
! udpsink host=192.168.2.108 port=5000 sync=false async=false
```

```
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp,encoding-name=VP8,payload=96 \
! rtpvp8depay \
! queue \
! avdec_vp8 \
! xvimagesink sync=false async=false
```

# h264
```
gst-launch-1.0 videotestsrc pattern=ball \
! 'video/x-raw, width=640, height=480, framerate=20/1' \
! omxh264enc control-rate=2 bitrate=4000000 preset-level=0 iframeinterval=100 \
! video/x-h264, stream-format=byte-stream \
! rtph264pay mtu=1400 \
! udpsink host=192.168.2.108 port=5000 sync=false async=false
```

```
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp,encoding-name=H264,payload=96 \
! rtph264depay \
! h264parse \
! queue \
! avdec_h264 \
! xvimagesink sync=false async=false -e
```


# Mixer
```
gst-launch-1.0 videomixer name=mix \
  sink_0::xpos=0 sink_0::ypos=0 \
  sink_1::xpos=400 sink_1::ypos=0 \
  sink_2::xpos=0 sink_2::ypos=300 \
  sink_3::xpos=400 sink_3::ypos=300 ! videoconvert ! autovideosink sync=false \
  videotestsrc ! video/x-raw,width=400, height=300 ! mix. \
  videotestsrc pattern=ball ! video/x-raw,width=400, height=300 ! mix. \
  videotestsrc pattern=bar ! video/x-raw,width=400, height=300 ! mix. \
  videotestsrc ! video/x-raw,width=400, height=300 ! mix.
```

```
gst-launch-1.0 videomixer name=mix \
  sink_0::xpos=0 sink_0::ypos=0 \
  sink_1::xpos=400 sink_1::ypos=0 \
  sink_2::xpos=0 sink_2::ypos=300 \
  sink_3::xpos=400 sink_3::ypos=300  ! videoconvert \
! omxh264enc bitrate=40000 preset-level=2 iframeinterval=100 \
! video/x-h264, stream-format=byte-stream \
! rtph264pay mtu=1400 \
! udpsink host=192.168.2.108 port=5000 sync=false async=false \
  videotestsrc ! video/x-raw,width=400, height=300,format=GRAY8 ! videoconvert ! mix. \
  videotestsrc pattern=ball ! video/x-raw,width=400, height=300 ! mix. \
  videotestsrc pattern=bar ! video/x-raw,width=400, height=300 ! mix. \
  videotestsrc ! video/x-raw,width=400, height=300 ! mix.
```