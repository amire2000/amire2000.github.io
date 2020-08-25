---
layout: post
title: Config jupyter notebook on jetson nano
categories: nvidia
tags: [nano, jupyter, opencvm vscode]
image: jetson-nano.png
public: true
description: Install and config jupyter notebook on jetson nano, config jupyter to work on local network , demo notebook use opencv
---
# LAB
Work on jetson nano from jupyter notebook
Run OpenCV and AI python code
Nono preinstall with OpenCV 4.3.0 with cuda support

LAB
- Install jupyter on jetson nano
- Access notebook from dev computer
  - Web access
  - VSCode
- Create venv on nano and install all components
- Demo
  - Read and display image
  - Play video
    - Stream Video using widget
    - Stream video using cvloop
    - Stream video using gstreamer to external console
  - Use CUDA
  -  
# Install
- create venv
- use pip to install

```bash
pip install wheel
pip install pyzmq
pip install jupyterlab
pip install cython
pip install numpy
pip install matplotlib
#install jupyter widgets
pip install ipywidgets
# strean webrtc
pip install ipywebrtc
```

create softlink to cv2.so

```
cd pyvenv/cvlab/lib/python3.6/site-packages
ln -s /usr/local/lib/python3.6/dist-packages/cv2 cv2
```

# config

```
jupyter notebook --generate-config
jupyter notebook password

```

# Run / notebook usage
```
jupyter notebook --ip=0.0.0.0
```

# vscode
- work with remote notebook

![](/images/2020-07-29-00-30-26.png)


# Demo
> jupyter show image in RGB format, opencv read image in BGR format

```python
#cell 1
import cv2
from matplotlib import pyplot as plt

#cell 2
img = cv2.imread("img/1.jpg")
# Convert img to RGB
img2 = img[:,:,::-1]
plt.imshow(img2)
plt.xticks([]), plt.yticks([])
plt.title("img1")
plt.show()

```

![](/images/2020-07-29-00-33-20.png)

## Demo2: Play video
> Install jupyter widgets

```python
from ipywidgets import Video, Image
from IPython.display import display
video = Video.from_file('movie.mp4')
video

# Other nb cell
# Clean widget resources
from ipywidgets import Widget
Widget.close_all()
```

![](/images/2020-08-07-08-11-04.png)

## Demo2a: Play video 
- capture Video or camera
- Run OpenCV algorithm
- Save output to Temp file
- Play the Output
- Clean Resources
  
```python
cap = cv2.VideoCapture('movie.mp4')

frames = []

while(1):
    try:
        _, frame = cap.read()

        fgmask = cv2.Canny(frame, 100, 100)

        mask = fgmask > 100
        frame[mask, :] = 0

        frames.append(frame)
    except Exception:
        break

width = int(cap.get(3))
height = int(cap.get(4))

filename = 'output.mp4'

fourcc = cv2.VideoWriter_fourcc(*'avc1')
writer = cv2.VideoWriter(filename, fourcc, 25, (width, height))

for frame in frames:
    writer.write(frame)

cap.release()
writer.release()

video = Video.from_file('output.mp4')
video
```

- Output video after `Canny`
![](/images/2020-08-07-15-41-38.png)

&nbsp;  
&nbsp;  
## Demo2b: Stream using gstreamer
```python
import cv2

cap = cv2.VideoCapture('movie.mp4')
width = int(cap.get(3))
height = int(cap.get(4))
out_pipe = "appsrc \
! autovideoconvert \
! omxh264enc control-rate=2 bitrate=400000 \
! video/x-h264, stream-format=byte-stream \
! rtph264pay mtu=1400 \
! udpsink host=192.168.10.1 port=5000 "
out = cv2.VideoWriter(out_pipe, cv2.CAP_GSTREAMER, 0, 10, (width,height), True)

while(1):
    try:
        _, frame = cap.read()
        out.write(frame)
        cv2.waitKey(1)
    except Exception:
        break
```
&nbsp;  
&nbsp;  
&nbsp;  
```
gst-launch-1.0 videotestsrc \
! video/x-raw,width=640,height=480 \
!  nvjpegenc \
! rtpjpegpay \
! udpsink host=127.0.0.1 port=5000
```

```
gst-launch-1.0 -v udpsrc port=5000 \
! application/x-rtp, encoding-name=JPEG,payload=26 \
! rtpjpegdepay \
! jpegdec \
! autovideosink
```
# Reference
- [Jupyter Notebook in Visual Studio Code](https://towardsdatascience.com/jupyter-notebook-in-visual-studio-code-3fc21a36fe43)
- [Video streaming in the Jupyter Notebook](https://towardsdatascience.com/video-streaming-in-the-jupyter-notebook-635bc5809e85)