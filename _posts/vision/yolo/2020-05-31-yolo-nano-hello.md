---
layout: post
title: YOLO jetson nano opencv and python
categories: vision
tags: [yolo, nvidia, opencv, python, jupyter, darknet]
image: 
description: Run YOLO algorithm on jetson nano, using opencv and darnet DNN with python binding
public: true
---

# Setup
> Install cuda and cudaDNN


```bash
# 
export PATH=/usr/local/cuda-10.0/bin:$PATH
#
# ADD cuda so to ld library path
```

## Clone darknet and download yolo
```
git clone https://github.com/AlexeyAB/darknet
cd darknet
wget https://pjreddie.com/media/files/yolov3.weights
wget https://pjreddie.com/media/files/yolov3-tiny.weights
```

## Build
- Enabled GPU support
- Make so library for python binding

```bash
vim Makefile
# edit makefile
GPU=1
CUDNN=1
OPENCV=1
LIBSO=1

# Run mke
make
```

## Check
```bash
./darknet detector test \
cfg/coco.data \
cfg/yolov3.cfg \
yolov3.weights \
-ext_output \
data/dog.jpg

# Result
Done! Loaded 107 layers from weights-file 
data/dog.jpg: Predicted in 709.568000 milli-seconds.
bicycle: 99%	(left_x:  117   top_y:  124   width:  452   height:  309)
dog: 100%	(left_x:  124   top_y:  224   width:  196   height:  320)
truck: 94%	(left_x:  474   top_y:   87   width:  217   height:   79)
```

## Install python binding
Python wrapper on YOLO 3.0 implementation by 'pjreddie' [link](https://pypi.org/project/yolo34py-gpu/)


```bash
# copy so to library folder or add path to ld.so.config
sudo cp libdarknet.so /usr/local/lib/
# install into virtual env
cd ~
python3 -m venv venv

pip install wheel
pip install yolo34py-gpu

# Create shortcut between venv and opencv so
cd venv/lib/python3.6/site-packages/
ln -s /usr/local/lib/python3.6/dist-packages/cv2/python-3.6/cv2.cpython-36m-aarch64-linux-gnu.so cv2.so

```

## Jupyter and demo code
- Run wrapper darknet DNN
- Install Jupyter into virtual env

```bash
source venv/bin/activate
pip install jupyterlab
```

![](/images/2020-05-31-07-30-00.png)

```python
from pydarknet import Detector, Image
from matplotlib import pyplot as plt
import cv2

net = Detector(bytes("/home/user/git/darknet/cfg/yolov3.cfg", encoding="utf-8"), bytes("/home/user/git/darknet/yolov3.weights", encoding="utf-8"), 0, bytes("/home/user/git/darknet/cfg/coco.data",encoding="utf-8"))

img = cv2.imread('/home/user/git/darknet/data/dog.jpg')

img_darknet = Image(img)
results = net.detect(img_darknet)

for cat, score, bounds in results:
    x, y, w, h = bounds
    cv2.rectangle(img, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (255, 0, 0), thickness=2)
    cv2.putText(img,str(cat.decode("utf-8")),(int(x),int(y)),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,0))

plt.imshow(img)
```

&nbsp;  
&nbsp;  
&nbsp;  
# Yolo OpenCV Implementation
```python
from matplotlib import pyplot as plt
import cv2

weights_file="/home/user/git/darknet/yolov3.weights"
cfg_file="/home/user/git/darknet/cfg/yolov3.cfg"
ds_file="/home/user/git/darknet/data/coco.names"

net = cv2.dnn.readNet(
    weights_file, 
    cfg_file)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
net.setPreferableBackend(cv2.dnn.DNN_TARGET_CPU)

import numpy as np
classes = []
with open(ds_file, "r") as f:
    classes = [line.strip() for line in f.readlines()]
# get output layers
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(len(classes), 3))

img = cv2.imread("person.jpg")
img = cv2.resize(img, None, fx=0.4, fy=0.4)
height, width, channels = img.shape

# Detecting objects
blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
net.setInput(blob)
outs = net.forward(output_layers)

# Showing informations on the screen
class_ids = []
confidences = []
boxes = []
for out in outs:
    for detection in out:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > 0.5:
            # Object detected
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[2] * width)
            h = int(detection[3] * height)
            # Rectangle coordinates
            x = int(center_x - w / 2)
            y = int(center_y - h / 2)
            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)
indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
font = cv2.FONT_HERSHEY_PLAIN
for i in range(len(boxes)):
    if i in indexes:
        x, y, w, h = boxes[i]
        label = str(classes[class_ids[i]])
        color = colors[i]
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        cv2.putText(img, label, (x, y + 30), font, 3, color, 3)

plt.imshow(img)
```


![](/images/2020-05-31-07-35-16.png)


&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Jupyter notebook config](https://jupyter-notebook.readthedocs.io/en/stable/public_server.html)
- [yolo34py-gpu](https://pypi.org/project/yolo34py-gpu/)