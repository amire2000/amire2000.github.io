---
layout: post
title: stream gstreamer in and out from opencv
categories: gstreamer
tags: [gstreamer, python, opencv, appsrc]
public: true
description: Gazebo camera sensor plugin , convert gazebo image to OpenCV Mat
---

# Stream from opencv
> opencv must be compile with gstreamer support, example cmake line
 ```
 -D WITH_V4L=ON \
 -D WITH_GSTREAMER=ON \
 -D WITH_GSTREAMER_0_10=OFF  
```

```python
import cv2
# print cv2.getBuildInformation()
stream = cv2.VideoWriter("appsrc \
    ! videoconvert \
    ! x264enc \
    ! h264parse \
    ! rtph264pay \
    ! udpsink host=127.0.0.1 port=5000 sync=false", 0, 10.0, (640,480))

capture = cv2.VideoCapture(0)

while True:
    ret,frame=capture.read()
    stream.write(frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
capture.release()
stream.release()
cv2.destroyAllWindows()
```

## Read stream 
```
gst-launch-1.0 udpsrc port=5000 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" \
! rtph264depay \
! decodebin \
! videoconvert \
! autovideosink
```

# Get gstreamer stream into opencv
- Run python app listen to h264 stream in port 5600 as default
- 
```python
import gi
from gi.repository import Gst
import cv2
import numpy as np

gi.require_version('Gst', '1.0')

class Video():
    def __init__(self, port=5600, test_src=False):
        Gst.init(None)

        self.port = port
        self._frame = None
        self._testsrc = test_src
        self.video_source = 'udpsrc port={}'.format(self.port)
        self.video_codec = '! application/x-rtp, payload=96 \
            ! rtph264depay \
            ! h264parse \
            ! avdec_h264'
        self.video_decode = '! decodebin \
            ! videoconvert \
            ! video/x-raw,format=(string)BGR \
            ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, pipe_elements):
        command = ' '.join(pipe_elements)
        self.video_pipe = Gst.parse_launch(command)
        
        self.video_sink = self.video_pipe.get_by_name('appsink0')
        self.video_sink.set_property('emit-signals',True)

    def frame(self):
        """ Get Frame
        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame


    def run(self):
        """ Get frame to update _frame
        """
        if self._testsrc:
            pipe_elements = \
            [
                'videotestsrc'
                '! decodebin',
                '! videoconvert'
                '! video/x-raw,format=(string)BGR '
                '! videoconvert',
                '! appsink'
            ]
        else:
            pipe_elements = \
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ]

        self.start_gst(pipe_elements)
        self.video_sink.connect('new-sample', self.callback)
        self.video_pipe.set_state(Gst.State.PLAYING)

    @staticmethod
    def gst_to_opencv(sample):
        """
        Transform byte array into np array
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK

    def frame_available(self):
        """Check if frame is available
        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)



if __name__ == '__main__':
    video = Video(test_src=False)

    while True:
        # Wait for the next frame
        if not video.frame_available():
            continue

        frame = video.frame()
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

```

- Stream webcam
```bash
gst-launch-1.0 -v v4l2src device="/dev/video1" \
! video/x-raw,framerate=20/1 \
! videoconvert \
! x264enc tune=zerolatency bitrate=500 speed-preset=superfast \
! rtph264pay \
! udpsink host=127.0.0.1 port=5600
```

## Docker notes
- Share web cam with docker container

```bash
docker run -it --rm \
--name cv \
--device=/dev/video1:/dev/video0 \
--user 0 \
-v <source>:/home/dev \
ae/opencv:344 /bin/bash
```

- Run docker as root for /dev/videoX permission or add user to `video` group

