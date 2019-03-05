
# First pipe
```
gst-launch-1.0 videotestsrc ! video/x-raw,width=640,height=480 ! autovideosink
```
- Read from camera
```
gst-launch-1.0 v4l2src device="/dev/video1" ! video/x-raw,width=640,height=480 ! videoconvert ! autovideosink
```

## Encoding
- mjpeg: jpegenc
- VP8: vp8enc
- Mpeg2: avenc_mpeg4

## Payload
packing the data into network protocol
- RTP: Real Time Prorocol
- GDP: Gstreamer Data Protocol

### Examples
- rtpkpegpay
- rtpv8pay
- rtpmp4vpay
- rtpvrawpay

Send stream
- udp
- tcp
- multiudpsink

source -> encode ->  pack -> send

recv -> unpack -> decode -> sink


### Mpeg 2 streaming
```
gst-launch-1.0 v4l2src device="/dev/video1" \
! video/x-raw,width=640,height=480 \
! videoconvert \
! avenc_mpeg4 \
! rtpmp4vpay \
! udpsink host=127.0.0.1 port=5600
```

```
gst-launch-1.0 -v udpsrc port=5600 \
! application/x-rtp,clock-rate=90000,payload=96 \
! rtpmp4vdepay \
! avdec_mpeg4 \
! autovideosink
```

## Networking 
- h264 codec
- rtp
```
gst-launch-1.0 videotestsrc pattern="ball" ! \
 video/x-raw,width=640,height=480 ! \
 videoconvert ! \
 x264enc ! \
 rtph264pay ! \
 udpsink host=127.0.0.1 port=5600
```
```
gst-launch-1.0 udpsrc port=5600 ! \
 application/x-rtp,encoding-name=H264 ! \
 rtph264depay ! \
 h264parse ! \
 avdec_h264 ! \
 autovideosink
```

## Mixer
```
gst-launch-1.0 videotestsrc pattern="ball" ! \
 video/x-raw,width=320,height=240 ! \
 videobox left=-320 border-alpha=0 ! \
 videomixer name="mix" ! \
 autovideosink \
 videotestsrc ! \
 video/x-raw,width=320,height=240 ! \
 mix.
```

### Mixer with network
```
gst-launch-1.0 videotestsrc pattern="ball" ! \
 video/x-raw,width=320,height=240 ! \
 videobox left=-320 border-alpha=0 ! \
 videomixer name="mix" ! \
 x264enc ! \
 rtph264pay ! \
 udpsink host=127.0.0.1 port=5600 \
 videotestsrc ! \
 video/x-raw,width=320,height=240 ! \
 mix.
```

-  not work to check
```
gst-launch-1.0 videomixer name=mix \
    sink_0::xpos=0   sink_0::ypos=0  sink_0::alpha=0 \
    sink_1::xpos=0   sink_1::ypos=0 \
    sink_2::xpos=200 sink_2::ypos=0 \
    autovideosink \
 videotestsrc pattern="ball" ! \
 videoconvert ! \
   mix.sink1 \
 videotestsrc ! \
 videoconvert ! \
  mix.sink2
```

## Add text to image
```
gst-launch-1.0 -v videotestsrc pattern="ball" !  video/x-raw,width=320,height=240 !  textoverlay text="CAM1"  font-desc="Sens 24" valignment=top halignment=left ! \
autovideosink
``` 
![](2019-02-28-21-53-45.png)

## Video scale
```
gst-launch-1.0 -v videotestsrc ! \
videoscale ! \
video/x-raw,width=100,height=100 ! \
autovideosink
```

## Video rate (fps)
```
gst-launch-1.0 -v videotestsrc pattern="ball" ! \
videorate ! \
video/x-raw,framerate=1/1 ! \
autovideosink
```

# Reference
- [NVIDIA Accelerated gstreamer user guide](https://usermanual.wiki/Document/AcceleratedGStreamerUserGuideRelease2421.1763245798/view)
- [RTP UDP](https://m.blog.naver.com/PostView.nhn?blogId=chandong83&logNo=221263551742&categoryNo=54&proxyReferer=https%3A%2F%2Fwww.google.com%2F)