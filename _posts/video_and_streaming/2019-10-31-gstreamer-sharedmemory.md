---
layout: post
title: Gstreamer SharedMemory
categories: video
tags: [gstreamer]
---


```
gst-launch-1.0 shmsrc socket-path=/tmp/foo ! \
avdec_h264 ! \
video/x-raw,width=640,height=480,framerate=25/1,format=YUY2 ! \
autovideosink
```

## source
```
gst-launch-1.0 videotestsrc is-live=true \
    ! jpegenc \
    ! rtpjpegpay \
    ! rtpstreampay \
    ! tcpserversink port=700
```

## dest

```
gst-launch-1.0 tcpclientsrc port=7001 \
    ! application/x-rtp-stream,encoding-name=JPEG \
    ! rtpstreamdepay \
    ! rtpjpegdepay \
    ! jpegdec \
    ! autovideosink
```


# h264 shm
## source
```
gst-launch-1.0 videotestsrc \
! x264enc \
! shmsink socket-path=/tmp/foo sync=true wait-for-connection=false shm-size=10000000

```

## dest
```
gst-launch-1.0 shmsrc socket-path=/tmp/foo \
    ! h264parse \
    ! avdec_h264 \
    ! videoconvert \
    ! autovideosink
```

# jpeg shm
## source
```
gst-launch-1.0 videotestsrc \
! jpegenc \
! shmsink socket-path=/tmp/foo \
sync=true \
wait-for-connection=false \
shm-size=10000000

```

## dest
```
gst-launch-1.0 shmsrc socket-path=/tmp/foo \
    ! jpegdec \
    ! videoconvert \
    ! autovideosink
```



# Reference
- [opencv server & client through shared memory](https://github.com/meirm/sharecam)
- [Python examples on how to use GStreamer within OpenCV ](https://github.com/madams1337/python-opencv-gstreamer-examples)


```

```