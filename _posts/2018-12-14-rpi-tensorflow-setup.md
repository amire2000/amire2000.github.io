---
layout: post
title: RPi Tensorflow setup for object detection
categories: tensorflow
tags: [tf, rpi, setup, object-detection]
---

## Download and flash Image
- Download raspbian from [Raspbian Stretch with desktop](https://www.raspberrypi.org/downloads/raspbian/)
- Flash to SD with [Ether](https://etcher.io/])

## Enabled SSH and wifi
### ssh
- Add empty file named `ssh` to `boot` partition

### enabled wireless
- Modify the file `wpa_supplicant.conf`

```bash
cd <rootfs>/etc/wpa_supplicant
sudo vim wpa_supplicant.conf
```

- Past at the end of file
```
network={
    ssid="<ssid>"
    psk="<pass>"
}
```

### First boot
- Connect with ssh
  - user:pi
  - pass:raspberry

- config `sudo raspi-config`
    - setup locals
    - changed password
    - resize rootfs partition

### Apt and  install
```
sudo apt update
sudo apt install vim
sudo apt install python3-picamera

```

### Resize swap file
```
sudo vim /etc/dphys-swapfile
# update
CONF_SWAPSIZE=1024
# save and exit
sudo /etc/init.d/dphys-swapfile stop
sudo /etc/init.d/dphys-swapfile start
#check
free -m
```

### Tensorflow

```
sudo apt install python3-pip
sudo apt install python3-dev

sudo apt install python-virtualenv

virtualenv --system-site-packages -p python3 tensorflow
source ~/tensorflow/bin/activate

pip install https://github.com/lhelontra/tensorflow-on-arm/releases/download/v1.11.0/tensorflow-1.11.0-cp35-none-linux_armv7l.whl

#
sudo apt-get install libatlas-base-dev
```

```
(tensorflow)$ python
>>> import tensorflow as tf
>>> tf.__version__
'1.11.0'


(tensorflow)pip install ipython
(tensorflow)python -m IPython

In [1]: import tensorflow as tf 
In [3]: hello=tf.constant("hello tf")                                                                
In [4]: with tf.Session() as s: 
   ...:     print (s.run(hello)) 
```

> Backup current image
> `sudo dd bs=4M if=/dev/sdb | gzip > ~/Downloads/rpi3tf111.img.gz `

### Object detection API
```
sudo apt install protobuf-compiler
source tensorflow/bin/activate
pip3 install pillow
pip3 install lxml
pip3 install jupyter
pip3 install matplotlib
pip3 install Cython
```

### Install OpenCV
```
sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install qt4-dev-tools

pip3 install opencv-python
```

### Compile and install Protobuf
All of TensorFlow's file formats are based on Protocol Buffers

```bash
sudo apt-get install autoconf automake libtool curl

#Download and extract
wget https://github.com/google/protobuf/releases/download/v3.5.1/protobuf-all-3.5.1.tar.gz
tar -zxvf protobuf-all-3.5.1.tar.gz
cd protobuf-3.5.1



# Make - take a long time
./configure
make
make check
sudo make all

cd python
export LD_LIBRARY_PATH=../src/.libs
python3 setup.py build --cpp_implementation 
python3 setup.py test --cpp_implementation
sudo python3 setup.py install --cpp_implementation

export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION_VERSION=3

sudo ldconfig

# Checking by run
protoc

##
sudo reboot now
```

## Download models



## Tools
### RPi camera
```
sudo apt install fswebcam
fswebcam image.jpg          #capture image from the camera
```
## Reference
- [EdjeElectronics tf object detection rpi](https://github.com/EdjeElectronics/TensorFlow-Object-Detection-on-the-Raspberry-Pi/tree/master)
- [Install TensorFlow on Raspberry Pi 3](https://www.deciphertechnic.com/install-tensorflow-on-raspberry-pi/)
- [libf77blas.so.3: cannot open shared object file: No such file or directory](https://github.com/Kitt-AI/snowboy/issues/262)
- [Object Detection on a Raspberry Pi](https://www.theta.co.nz/news-blogs/tech-blog/object-detection-on-a-raspberry-pi/)
- [tensorflow models](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md)
- [EdjeElectronics/TensorFlow-Object-Detection-on-the-Raspberry-Pi](https://github.com/EdjeElectronics/TensorFlow-Object-Detection-on-the-Raspberry-Pi)
- [Odroid XU4 tensorflow](https://www.jianshu.com/p/aa57435baa8a)
- [Intro - TensorFlow Object Detection API Tutorial p.1](https://www.youtube.com/watch?v=COlbP62-B-U)
- [Compiling TensorFlow Lite for a Raspberry Pi](https://medium.com/@haraldfernengel/compiling-tensorflow-lite-for-a-raspberry-pi-786b1b98e646)
- [Odroid XU4 Tensorflow](https://forum.odroid.com/viewtopic.php?f=95&t=28177#p202449)