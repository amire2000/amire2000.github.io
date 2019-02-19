---
layout: post
title: Video encoding
categories: media
tags: [video, encoding, gstreamer]
---

### Format
### Codec: enCoder/Decoder
Method of compression
- H.264
- H.265
- VP9



### Delivery Modes
- Streaming: 
- Progressive Download: Download the video to local and than play it's


### Example
- Simple test
```bash
sudo gst-launch-1.0 -v videotestsrc ! autovideosink
```

### Terms
Bit rate: Size of the video file per second
CBR: Constant Bitrate
VBR: Variable Bitrate


## Reference 
- [Gstreamer cheat sheet](http://wiki.oz9aec.net/index.php/Gstreamer_cheat_sheet)
- [Install GStreamer](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html)