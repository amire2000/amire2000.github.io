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

- x264: open source  implementation for H.264 codec
  
### Network protocols
- RTSP: Real Time Streaming Protocol, **Control** streaming server action like PLAY STOP, 
**Raw UDP** or **RTP** over udp are transmit the stream

- RTP: Real Time Protocol, delivering audio and video over ip

- RTCP: RTP Control Protocol: monitor transmission statistics and QoS

### API
- WebRTC: Web Real-Time Communication: API  to create RTP application
- GStreamer: 
- FFmpeg: A complete, cross-platform solution to record, convert and stream audio and video ([link](www.ffmpeg.org))
- 
### Compression
- Inter-Frame: Compress each frame (as jpeg for example - Motion-JPEG)
- Intra-Frame: Compression the difference im images (H.264, XVID Divx)

### Terms
- Bit rate: Size of the video file per second
- CBR: Constant Bitrate
- VBR: Variable Bitrate - Intra-Frame compression produce variable bit rate
- Bitrate: Describe the rate at witch bit are transferred from one location to another
(bps: bits per seconds)
- latency: is amount  ot time between from capture and when it's display
- DSB: Decoder Stream Buffer

![](.2019-02-17-video-encodeing-101_images/3afc5487.png)

### Delivery Modes
- Streaming: 
- Progressive Download: Download the video to local and than play it's


### Example
- Simple test
```bash
sudo gst-launch-1.0 -v videotestsrc ! autovideosink
```





## Reference 
- [Gstreamer cheat sheet](http://wiki.oz9aec.net/index.php/Gstreamer_cheat_sheet)
- [Install GStreamer](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html)