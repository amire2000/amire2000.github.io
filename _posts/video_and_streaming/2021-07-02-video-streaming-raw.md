---
layout: post
title: Raw Video transport
categories: video
tags: [video, streaming, raw]
public: true
description: Video RAW transfer between components
---

Compare ways to transfer RAW image between process in the same computer

- using gstreamer (udp)
- using opencv / numpy zmq as transport

## Lab settings

- Camera: Logitech c920
- Resolution: 640\*480
- RGB
- FPS: 30

## Gstreamer

```bash
# script: capture_view_basic
gst-launch-1.0 v4l2src device=/dev/video2 \
! video/x-raw,width=640,height=480,framerate=30/1 \
! videoconvert \
! fpsdisplaysink

# script: capture_fake
gst-launch-1.0 v4l2src device=/dev/video2 \
! video/x-raw,width=640,height=480,framerate=30/1 \
! videoconvert \
! fpsdisplaysink


# script: raw_udp
gst-launch-1.0 v4l2src device=/dev/video2 \
! video/x-raw,width=640,height=480,framerate=30/1 \
! videoconvert \
! rtpvrawpay \
! udpsink host=127.0.0.1 port=5001 sync=false async=false

# recv
# script: raw_udp_recv
gst-launch-1.0 udpsrc port=5001 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=(string)YCbCr-4:2:2, depth=(string)8,width=(string)640, height=(string)480, colorimetry=(string)BT601-5, payload=(int)96, ssrc=(uint)155528026, timestamp-offset=(uint)2270520902, seqnum-offset=(uint)27437,a-framerate=(string)30" \
! rtpvrawdepay \
! videoconvert \
! queue \
! fpsdisplaysink sync=false async=false

```

| #                             | CPU (%) |
| ----------------------------- | ------- |
| capture_view_basic            | 4-4.5   |
| capture_fake                  | <1      |
| raw_udp_sender (without recv) | 21      |
| raw_udp_sender (with recv)    | 10      |
| raw_udp_recv                  | 15      |

> Check why without running receiver sender usage more cpu
# Check latency

```bash
# Sender
gst-launch-1.0 v4l2src device=/dev/video2 \
! video/x-raw,width=640,height=480,framerate=30/1 \
! videoconvert \
! rtpvrawpay \
! udpsink host=127.0.0.1 port=5001 sync=false async=false

# Sender with tee
gst-launch-1.0 v4l2src device=/dev/video2 \
! video/x-raw,width=640,height=480,framerate=30/1 \
! timeoverlay \
! tee name=t \
t. ! queue ! videoconvert ! fpsdisplaysink sync=false async=false \
t. ! queue \
! videoconvert \
! rtpvrawpay \
! udpsink host=127.0.0.1 port=5001 sync=false async=false

# script: raw_udp_recv
gst-launch-1.0 udpsrc port=5001 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=(string)YCbCr-4:2:2, depth=(string)8,width=(string)640, height=(string)480, colorimetry=(string)BT601-5, payload=(int)96, ssrc=(uint)155528026, timestamp-offset=(uint)2270520902, seqnum-offset=(uint)27437,a-framerate=(string)30" \
! rtpvrawdepay \
! videoconvert \
! queue \
! fpsdisplaysink sync=false async=false
```

![](/images/2021-07-02-11-41-33.png)

| #                          | CPU (%) |
| -------------------------- | ------- |
| sender (without time over) | 10      |
| sender (tee)               | 16      |
| recv (fpsdisplaysink)      | 13      |

&nbsp;  
&nbsp;  
&nbsp;

# Numpy msgpack and ZMQ

- Using zmq as transport
- Using msgpack to serial

### Generate image with time stamp

{% gist 8ddf66e69b9f2bfe05e778ea9198e71a %}

&nbsp;  
&nbsp;  
&nbsp;

## ZMQ Pub/Sub

- full code below
- FPS: 30
- size: 640\*480
  &nbsp;

![](/images/2021-07-02-17-53-29.png)

| #                              | CPU (%) |
| ------------------------------ | ------- |
| sender (without frame display) | 8.5     |
| recv                           | 12      |

```python

import multiprocessing
import cv2
import numpy as np
import zmq
import time
from timeit import default_timer as timer

start = timer()
TOPIC = b"topic"

HEIGHT=480
WIDTH=640
CHANNELS=3

def create_image() -> np.ndarray:
    FONT                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (100,100)
    FONT_SCALE=2
    fontColor              = (0,255,0)
    lineType               = 2
    frame = np.zeros((HEIGHT, WIDTH, CHANNELS), np.uint8)
    end = timer()
    delta_sec = round(end-start, 3)
    cv2.putText(frame,str(delta_sec),
        bottomLeftCornerOfText,
        FONT,
        FONT_SCALE,
        fontColor,
        lineType)

    return frame

def pub(name="pub", show=False):
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5555")
    print('Binding to port 5555')
    while True:
        frame = create_image()
        data = frame.tobytes()
        socket.send_multipart((TOPIC, data))
        if show:
            cv2.imshow(name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        time.sleep(1/30)


def sub(name="sub", show=False):
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://127.0.0.1:5555")
    socket.setsockopt(zmq.SUBSCRIBE, TOPIC)
    while True:
        topic, data = socket.recv_multipart()
        frame = np.frombuffer(data, dtype=np.uint8)
        frame = frame.reshape((HEIGHT, WIDTH, CHANNELS))
        if show:
            cv2.imshow(name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    SUB_SHOW_IMG = True
    p_pub = multiprocessing.Process(target=pub, args=("pub", ))
    p_sub = multiprocessing.Process(target=sub, args=("sub1", SUB_SHOW_IMG))
    p_sub2 = multiprocessing.Process(target=sub, args=("sub2", SUB_SHOW_IMG))
    p_sub.start()
    p_sub2.start()
    p_pub.start()

    p_pub.join()
    p_sub.join()
    p_sub2.join()
```

latency with tow subscribers
![](/images/2021-07-02-22-39-53.png)

&nbsp;  
&nbsp;  
&nbsp;

## ZMQ Pub/Sub ,msgpack and custom dataclass as msg

- Add custom msg to holed image data
- Use ZMQ Pub/Sub
  - Test with two subscribers
- Use msgpack as serializer
- Write custom encoder/decoder to pack `dataclass`

| #                              | CPU (%) |
| ------------------------------ | ------- |
| sender (without frame display) | 7-8     |
| recv 1                         | 12-14   |
| recv 2                         | 12-14   |
| recv 1 (without frame display) | ~6      |
| recv 2 (without frame display) | ~6      |

![](/images/2021-07-03-08-30-57.png)
&nbsp;  
&nbsp;

- code example

```python
import multiprocessing
import cv2
import numpy as np
import zmq
import time
import msgpack
from timeit import default_timer as timer
from dataclasses import dataclass

def encode(obj):
    return{
        b'w': obj.width,
        b'h': obj.height,
        b'c': obj.channels,
        b'data': obj.img,
    }

def decode(obj):
    return ImgMsg(
        width=obj[b'w'],
        height=obj[b'h'],
        channels=obj[b'c'],
        img=obj[b'data']
    )

@dataclass
class ImgMsg():
    width: int
    height: int
    channels: int
    img: bytes

start = timer()
TOPIC = b"topic"

def create_image() -> np.ndarray:
    HEIGHT=480
    WIDTH=640

    FONT                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (100,100)
    FONT_SCALE=2
    fontColor              = (0,255,0)
    lineType               = 2
    frame = np.zeros((HEIGHT, WIDTH,3), np.uint8)
    end = timer()
    delta_sec = round(end-start, 3)
    cv2.putText(frame,str(delta_sec),
        bottomLeftCornerOfText,
        FONT,
        FONT_SCALE,
        fontColor,
        lineType)

    return frame

def pub(name="pub", show=False):
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5555")
    print('Binding to port 5555')
    while True:
        frame = create_image()
        # data = msgpack.packb(frame.tobytes())
        c = 1
        if len(frame.shape) == 2:
            h, w = frame.shape
        else:
            h, w, c = frame.shape

        msg:np.ndarray = ImgMsg(
            width=w,
            height=h,
            channels=c,
            img=frame.tobytes()
            )
        data = msgpack.packb(msg, default=encode)
        socket.send_multipart((TOPIC, data))
        if show:
            cv2.imshow(name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        time.sleep(1/30)


def sub(name="sub", show=False):
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://127.0.0.1:5555")
    socket.setsockopt(zmq.SUBSCRIBE, TOPIC)
    while True:
        topic, data = socket.recv_multipart()
        # data = msgpack.unpackb(data)
        msg = msgpack.unpackb(data, object_hook=decode)
        frame = np.frombuffer(msg.img, dtype=np.uint8)
        frame = frame.reshape((msg.height, msg.width, msg.channels))
        if show:
            cv2.imshow(name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    SUB_SHOW_IMG = True
    p_pub = multiprocessing.Process(target=pub, args=("pub", ))
    p_sub = multiprocessing.Process(target=sub, args=("sub1", SUB_SHOW_IMG))
    p_sub2 = multiprocessing.Process(target=sub, args=("sub2", SUB_SHOW_IMG))
    p_sub.start()
    p_sub2.start()
    p_pub.start()

    p_pub.join()
    p_sub.join()
    p_sub2.join()
```
