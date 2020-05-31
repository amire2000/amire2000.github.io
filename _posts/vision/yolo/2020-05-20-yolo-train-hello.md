---
layout: post
title: Train YOLOv3 detector from scratch
categories: vision
tags: [yolo, train]
public: true
description: 
image: 
---
# YOLO
Is a method (way) to do object detection. It is the algorithm how to detect objects in the image.

# OpenCV dnm module
DNN (Deep Neural Network) module allow opencv to run inference in pre-trained modules


# Object detection with pre-trained YOLOv3 weights
- Ubuntu 20.04
- OpenCV 4.2 (Install from ubuntu repo)
- CPU as backend
- Download 
  - [yolov3.cfg](https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg)
  - [weights](https://pjreddie.com/darknet/yolo/)
  - [COCO dataset](https://opencv-tutorial.readthedocs.io/en/latest/_downloads/a9fb13cbea0745f3d11da9017d1b8467/coco.names)


## Project struct
```
├── coco.names
├── demo1.py
├── person.jpg
├── yolov3.cfg
└── yolov3.weights
```
&nbsp;  
&nbsp;  
## Code demo
```python
import cv2
import numpy as np
import os

loc = os.path.dirname(__file__)
weights_file = os.path.join(loc, "yolov3.weights")
cfg_file = os.path.join(loc, "yolov3.cfg")
ds_file = os.path.join(loc, "coco.names")

# Load Yolo
net = cv2.dnn.readNet(
    weights_file, 
    cfg_file)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
net.setPreferableBackend(cv2.dnn.DNN_TARGET_CPU)
# Load class from coco
classes = []
with open(ds_file, "r") as f:
    classes = [line.strip() for line in f.readlines()]
# get output layers
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(len(classes), 3))

# Loading image
img = cv2.imread(os.path.join(loc, "person.jpg"))
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


cv2.imshow("Image", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

![](/images/2020-05-22-18-47-29.png)
&nbsp;  
&nbsp;  
&nbsp; 

# Setup
## VoTT
Visual Object Tagging Tool  

- Dowmload snap file [download](https://github.com/Microsoft/VoTT/releases)

```bash
#ignore missing signature
sudo snap install vott-2.1.0-linux.snap --dangerous
```

&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [How to train your own YOLOv3 detector from scratch](https://blog.insightdatascience.com/how-to-train-your-own-yolov3-detector-from-scratch-224d10e55de2)
- [YOLO 4](https://robocademy.com/2020/05/01/a-gentle-introduction-to-yolo-v4-for-object-detection-in-ubuntu-20-04/)
- [Train YOLO to detect a custom object (online with free GPU)](https://pysource.com/2020/04/02/train-yolo-to-detect-a-custom-object-online-with-free-gpu/)
- [YOLO V3 – Install and run Yolo on Nvidia Jetson Nano (with GPU)](https://pysource.com/2019/08/29/yolo-v3-install-and-run-yolo-on-nvidia-jetson-nano-with-gpu/)