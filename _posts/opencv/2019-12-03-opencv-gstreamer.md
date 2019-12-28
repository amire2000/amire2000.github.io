---
layout: post
title: Read and Write into gstreamer pipe
categories: opencv
tags: [gstreamer]
public: true
description: using opencv with gstreamer, read from pipe using gst appsink, write  into pipe using appsrc
---
- Check that opencv support gstreamer with `cv2.getBuildInformation()` method

![](/images/2019-12-03-23-16-48.png)

# Simple demo
- appsink -> opencv using VideoCapture
- opencv -> appsrc using VideoWriter


```python
import cv2

cap = cv2.VideoCapture("videotestsrc ! video/x-raw,width=640,height=480,framerate=10/1 ! videoconvert ! timeoverlay ! appsink")
out_pipe = "appsrc ! video/x-raw,width=640,height=480,framerate=10/1 ! videoconvert ! timeoverlay xpad=100 ypad=100 ! autovideosink sync=false"
out = cv2.VideoWriter(out_pipe, 0, 10.0, (640,480))

while True:
    ret, frame = cap.read()
    cv2.imshow("cv", frame)
    out.write(frame)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
cap.release()
out.release()
cv2.destroyAllWindows()
```

```python
import cv2

cap_str = '-v udpsrc port=1234 caps="application/x-rtp,media=(string)video,encoding-name=(string)H264" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink'
cap = cv2.VideoCapture(cap_str)

while True:
    ret, frame = cap.read()
    cv2.imshow("cv", frame)

    if cv2.waitKey(1) & 0xff == ord('q'):
        break
cap.release()

cv2.destroyAllWindows()
```

```bash
 gst-launch-1.0 -v udpsrc port=1234 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
 ```
&nbsp;  
&nbsp;  
&nbsp;  
## Cpp
```cpp
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main()
{
  cout << "OpenCV version : " << CV_VERSION << endl;
  
 
    VideoCapture cap(1);
    if (!cap.isOpened()) {
        cerr <<"VideoCapture not opened"<<endl;
        exit(-1);
    }

	VideoWriter writer(
        "appsrc ! video/x-raw,width=640,height=480,framerate=10/1 ! videoconvert ! timeoverlay xpad=100 ypad=100 ! autovideosink sync=false",
		// "appsrc ! videoconvert ! video/x-raw,width=640,height=480,framerate=10/1 ! jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=5000", 
        0,		// fourcc 
		10,		// fps
		Size(640, 480), 
		true);	// isColor

    if (!writer.isOpened()) {
        cerr <<"VideoWriter not opened"<<endl;
        exit(-1);
    }

    while (true) {
        Mat frame;
        cap.read(frame);
        writer.write(frame);
    }

    return 0;
}
```

- meson

```python
#/usr/lib/x86_64-linux-gnu/pkgconfig/opencv.pc
opencv = dependency('opencv', version : '>=4')
# compiler = meson.get_compiler('cpp')
# deps = [compiler.find_library('cv2', dirs : ['/home/user/opencv/opencv/build/lib'])]
install_dir = meson.source_root() + '/bin'
incdir = include_directories('/home/user/opencv/opencv/build/include')
src = ['cv2gst.cpp']

executable('cv2gst', 
    sources : src, 
    dependencies : opencv,
    install: true,
    install_dir: [install_dir])
```
# Reference 
- [demo](https://qiita.com/satoyoshiharu/items/72a540a92578faa7929d)