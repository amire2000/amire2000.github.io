---
layout: post
title: Using qt to view image stream
categories: python
tags: [qt, stream, opencv, pyside2]
description: Using QT to view image stream
public: true
image: qtpython.png
---

# Content
    - Qt basic
    - Qt load image into label
    - Qt capture camera

&nbsp;  
&nbsp;  
# Qt basic

```python
from PySide2.QtWidgets  import QApplication, QLabel
from PySide2 import QtCore, QtGui, QtWidgets
import sys

if __name__ == '__main__':
    app = QApplication(sys.argv)
    label = QLabel()
    label.setText("hello world")
    label.resize(640, 480)
    label.show()
    sys.exit(app.exec_())
```
&nbsp;  
&nbsp;  
# Qt load image
```python
from PySide2.QtWidgets  import QApplication, QLabel
from PySide2 import QtCore, QtGui, QtWidgets
import cv2
import sys

ROWS = 0
COLS = 1

if __name__ == '__main__':
    app = QApplication(sys.argv)
    label = QLabel()
    img = cv2.imread("/home/user/projects/py_tutorial/src/ffmpeg/1.jpg")
    label.resize(640, 480)
    label.setPixmap(QtGui.QPixmap.fromImage(QtGui.QImage(img.data,
        img.shape[COLS],
        img.shape[ROWS],
        QtGui.QImage.Format_RGB888)))
    label.show()
    sys.exit(app.exec_())
```
&nbsp;  
&nbsp;  
# Qt capture img from camera
```python
from PySide2.QtWidgets  import QApplication, QLabel
from PySide2 import QtCore, QtGui, QtWidgets
import cv2
import sys
import threading
import time
import numpy as np

ROWS = 0
COLS = 1

def capture():
    try:
        cap = cv2.VideoCapture(1)
        while True:
            _, frame = cap.read()
            frame[:,:,0] = frame[:,:,0].astype(np.float32) 
            frame[:,:,1] = frame[:,:,1].astype(np.float32) 
            frame[:,:,2] = frame[:,:,2].astype(np.float32)

            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            label.setPixmap(QtGui.QPixmap.fromImage(QtGui.QImage(img.data,
                img.shape[COLS],
                img.shape[ROWS],
                QtGui.QImage.Format_RGB888)))
            time.sleep(1/30)
    except Exception  as e:
        print(e)
        

if __name__ == '__main__':
    

    app = QApplication(sys.argv)
    label = QLabel()
    img = cv2.imread("/home/user/projects/py_tutorial/src/ffmpeg/1.jpg")
    label.resize(640, 480)
    label.setPixmap(QtGui.QPixmap.fromImage(QtGui.QImage(img.data,
        img.shape[COLS],
        img.shape[ROWS],
        QtGui.QImage.Format_RGB888)))
    
    t1 = threading.Thread(target=capture)
    t1.setDaemon(True)
    t1.start()
    
    label.show()
    sys.exit(app.exec_())
```
# Reference
