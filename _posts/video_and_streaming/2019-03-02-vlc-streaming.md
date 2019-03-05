

```
cvlc -vvv v4l2:///dev/video1 --sout '#transcode{vcodec=mp4v,vb=800,acodec=none}:rtp{sdp=rtsp://:8554/}'

vlc -vvv --network-caching 200 rtsp://127.0.0.1:8554/
```

```

```