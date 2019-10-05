---
layout: post
title: FFMPEG Streaming
categories: video
tags: [ffmpeg, ffserver, mjpeg]
image: streaming.png
description: Using ffmpeg and ffserver to stream video
public: true
---

# Streaming using RTP
```
ffmpeg -re -f lavfi -i aevalsrc="sin(400*2*PI*t)" -ar 8000 -f mulaw -f rtp rtp://127.0.0.1:1234

ffplay rtp://127.0.0.1:1234
```

# Stream using mpeg
```bash
ffmpeg \
  -f lavfi \
  -i testsrc=size=800x600:rate=24 \
  -c:v libx264 \
  -tune zerolatency \
  -pix_fmt yuv420p \
  -profile:v baseline \
  -b:v 300K \
  -minrate 100K \
  -maxrate 1M \
  -bufsize 2M \
  -g 24 \
  -f mpegts \
  tcp://127.0.0.1:1234?listen
#play
  vlc tcp://@127.0.0.1:1234

```

# FFServer
- ffserver receives prerecorded files or FFM streams from some ffmpeg instance as input, then streams them over RTP/RTSP/HTTP.

- Start `ffserver`
- Start `ffmpeg`
- Run `player`
  
## server.conf
```
HTTPPort            8090
HTTPBindAddress     0.0.0.0
MaxHTTPConnections 200
MaxClients      100
MaxBandWidth    500000
CustomLog       -

<Feed camera.ffm>
File            /tmp/camera.ffm
FileMaxSize     200M
</Feed>

<Stream camera.mjpeg>
Feed camera.ffm
Format mpjpeg
VideoFrameRate 15
VideoIntraOnly
VideoBitRate 4096
VideoBufferSize 4096
VideoSize 640x480
VideoQMin 5
VideoQMax 51
NoAudio
Strict -1
</Stream>
```

## Usage
### Terminl 1 (run server)
```
ffserver -f server.conf
```
&nbsp;  
&nbsp;  
### Terminal 2 (run ffmpeg)
  
```
ffmpeg \
-f video4linux2 \
-s 640x480 \
-r 15 \
-i /dev/video0 \
http://localhost:8090/camera.ffm
```

- -f: source format
- -s: size
- -r: rate
- -i: input
&nbsp;  
&nbsp;  
### Terminal 3 (vlc)
```
http://localhost:8090/camera.mjpeg
```


# Reference 
- [Streaming guide](https://trac.ffmpeg.org/wiki/StreamingGuide)
- [Testing vido source](https://www.bogotobogo.com/FFMpeg/ffmpeg_video_test_patterns_src.php)
- [nvidia ffmpeg](https://devblogs.nvidia.com/nvidia-ffmpeg-transcoding-guide/)
- [Live video streaming with ffmpeg and ffserver](http://www.budthapa.pro/2017/05/live-video-streaming-with-ffmpeg-and.html)
- [Learn to Produce Video with FFmpeg in 30 Minutes](https://streaminglearningcenter.com/learnffmpeg)
- [Multimedia Technology Basics](https://multimedia.cx/mmbasics.txt)
