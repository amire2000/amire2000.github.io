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
  - Use CUDA
  -  
# Install
- create venv
- use pip to install

```
pip install wheel
pip install pyzmq
pip install jupyterlab
pip install cython
pip install numpy
pip install matplotlib
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

## Demo2: convert image to gray using cuda
```python
```
&nbsp;  
&nbsp;  
&nbsp;  

# Reference
- [Jupyter Notebook in Visual Studio Code](https://towardsdatascience.com/jupyter-notebook-in-visual-studio-code-3fc21a36fe43)