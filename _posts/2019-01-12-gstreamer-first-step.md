---
layout: post
title: gstreamer first step
categories: gstreamer
tags: [gstreamer]
---

![](/images/2019-01-13-19-55-07.png)

## Tools
- gst-inspect: discover gstreamer  elements
- gst-launch: build a  gstreamer pipeline on the command line

```
gst-launch filesrc location=~/Music/sample.mp3 ! decodebin ! audioconvert ! autoaudiosink

gst-launch filesrc location=~/Music/sample.mp3 ! decodebin ! audioconvert ! wavenc ! filesink location=~/Music/sample.wav

gst-launch filesrc location=~/Music/sample.wav ! decodebin ! audioconvert ! autoaudiosink
```

### Video
```
gst-launch-1.0 -v v4l2src device=/dev/video1 ! videorate ! video/x-raw,framerate=5/1,width=640,height=480 ! xvimagesink
```

### Networking
```bash
#sender
gst-launch-1.0 -v v4l2src device=/dev/video1 ! video/x-raw,width=640,height=480 !  jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=1234

#receiver
gst-launch-1.0 -v udpsrc port=1234 ! application/x-rtp, encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink

```

##  Reference
- [Gstreamer cheat sheet](http://wiki.oz9aec.net/index.php/Gstreamer_cheat_sheet#Network_Streaming)
- [GStreamer-1.0 personal cheat sheet](https://gist.github.com/strezh/9114204)
- [Play webcam using gstreamer](https://medium.com/@petehouston/play-webcam-using-gstreamer-9b7596e4e181)
- [Videostreaming with Gstreamer](http://z25.org/static/_rd_/videostreaming_intro_plab/index.html)
- [Wide Area Network Emulator](https://sourceforge.net/projects/wanem/)