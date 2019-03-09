---
layout: post
title: VLC cheat-sheet
categories: media
tags: [video, vlc]
---

```
cvlc -vvv v4l2:///dev/video1 --sout '#transcode{vcodec=mp4v,vb=800,acodec=none}:rtp{sdp=rtsp://:8554/}'

vlc -vvv --network-caching 200 rtsp://127.0.0.1:8554/
```

```

```

## Reference
- [Live Webcam Streaming using VLC on the Command Line](https://sandilands.info/sgordon/live-webca-streaming-using-vlc-command-line)