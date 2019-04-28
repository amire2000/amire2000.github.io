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
- Bitrate: Describe the rate at witch bit are transferred from one location to another
  - Rate at which data is encoded per second
(bps: bits per seconds)
- CBR: Constant Bitrate
- VBR: Variable Bitrate - Intra-Frame compression produce variable bit rate

- latency: is amount  ot time between from capture and when it's display
- DSB: Decoder Stream Buffer

## H264
Called AVC or H.264 (MPEG-4 Part 10)
x264 - open source implementation for H264

.MP4, .MOV .M4V ... are all containers to data encode in H.264

- I frame: (Intra-coded picture) is a complete image
- P frame: (Predicted picture) holds only the changes in the image from the previous frame.
- B frame: (Bidirectional predicted picture) saves even more space by using differences between the current frame and both the preceding and following frames to specify its content.

### Most common parameters
- Profiles
  - Baseline
  - Main
  - High
  - ...
- B-frames/ I-frames
- Entropy encoding

> Typical B-Frame encoding parameter (2-3)
> Typical B-Frame search (Reference frame) (3-5)
> IBBBPBBBP

### Gstreamer x264enc
- bframe: Number of B-frames between I and P
- cabac: boolean, enabled entropy CABAC 
- ref: number of reference frames(1,12)
  
### Delivery Modes
- Streaming: 
- Progressive Download: Download the video to local and than play it's

### Entropy Encoding
- CABAC (prepared)
- CAVLC



### Example
- Simple test
```bash
sudo gst-launch-1.0 -v videotestsrc ! autovideosink
```





## Reference 
- [Gstreamer cheat sheet](http://wiki.oz9aec.net/index.php/Gstreamer_cheat_sheet)
- [Install GStreamer](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html)