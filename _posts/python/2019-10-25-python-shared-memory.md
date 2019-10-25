---
layout: post
title: Python shared memory
categories: python
tags: [shared-memory]
description: Using shared memory to shared image / numpy array between process
public: true
image: shared-memory.jpg
---
Shared memory is a concept where two or more process can access the common memory.

- shmget: System V shared memory
- mmap: POSIX way


# Python
## SharedArray library
[pip SahredArrayl](https://pypi.org/project/SharedArray/)  
Create shared memory as numpy array

### install

```
pip install SharedArray
```

### create
This function creates an array in shared memory and returns a numpy array that uses the shared memory as data backend.

```
SharedArray.create(name, shape, dtype=float)
```

The content of the array lives in shared memory and/or in a file and won’t be lost when the numpy array is deleted, nor when the python interpreter exits. To delete a shared array and reclaim system resources use the `SharedArray.delete()` function

The function create a file in `/dev/shm` folder


### Demo
shared image between two process

#### Pub
```python
import cv2
import numpy as np
import SharedArray as sa
WIN_NAME = "pub"
SHM_NAME = "shm://cv_image"
cam = cv2.VideoCapture(1, cv2.CAP_V4L2)
shape = (480, 640, 3)
shm = sa.create(SHM_NAME, shape, dtype=np.uint8)


while True:
    ret, frame = cam.read()
    shm[:] = frame
    cv2.imshow(WIN_NAME, frame)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cam.release()
#sa.delete(SHM_NAME)
cv2.destroyAllWindows()
```

### sub

```python
import cv2
import numpy as np
import SharedArray as sa
WIN_NAME = "sub"
SHM_NAME = "shm://cv_image"
shape = (640, 480, 3)
shm = sa.attach(SHM_NAME)

while True:
    cv2.imshow(WIN_NAME, shm)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

sa.delete(SHM_NAME)
cv2.destroyAllWindows()
```

# Reference
- [IPC:Shared Memory](http://users.cs.cf.ac.uk/Dave.Marshall/C/node27.html)

