---
layout: post
title: Openvino RaspberryPi setup
categories: OpenVino
tags: [openvino, intel, opencv, rpi, setup]
description: Setup intel Neural compute stick with raspberry pi 3+, Install toolkit and run sample application 
public: true
image: openvino_logo.png
---
<style>
    img[src*='#logo'] {
    max-width:200px;
}
    
</style>


![](/images/open_vino_rpi_install.jpg)

## Objectives
- Install Raspbian OS (lite for me)
- Install OpenVino toolkit
- Setup the environment
- Run sample application

## Hardware
- RPI3+
- microSD (32G)
- Intel Neural Compute Stick 2
- USB3 extension cable (Stick block all other usb ports)


The OpenVINO™ toolkit for Raspbian* OS includes the Inference Engine and the MYRIAD plugin only, The toolkit install opencv 4.0 compiled for intel hardware

## Image flash
- Download Flash Raspbian stretch (lite for me) to microSD card using [Etcher](https://www.balena.io/etcher/)

![](/images/2019-05-07-15-19-14.png#logo)

## OS setup
![](/images/raspbian_logo.png)
- Enabled ssh
- Os setup

### Enabled ssh
Create empty file named `ssh` in the boot partition

### OS Setup
ssh into pi: user:pi password: raspberry
```bash
sudo raspi-config
```
- Set local (Localization Options)
- Expand FS: Advanced Option -> Expand Filesystem   

## Openvino toolkit
![](/images/openvino_logo.png)
### Install dependencies
```
sudo apt update
sudo apt install cmake
```
### Download toolkit
- Download last version `https://download.01.org/opencv`
  
```bash
cd ~/Download
wget --no-check-certificate https://download.01.org/opencv/2019/openvinotoolkit/l_openvino_toolkit_raspbi_p_2019.1.094.tgz
```

### Install toolkit
```bash
#create install folder
sudo mkdir -p /opt/intel/openvino
sudo tar -xf l_openvino_toolkit_raspbi_p_2019.1.094.tgz.tgz --strip 1 -C /opt/intel/openvino
# Update setupvars.sh INSTALLDIR variable
# part of setupvars.sh file
# INSTALLDIR=<INSTALLDIR>
# export INTEL_OPENVINO_DIR=$INSTALLDIR
sudo sed -i "s|<INSTALLDIR>|/opt/intel/openvino|" /opt/intel/openvino/bin/setupvars.sh

# add setupvars.sh to .bashrc
echo "source /opt/intel/openvino/bin/setupvars.sh" >> ~/.bashrc
# check
source ~/.bashrc 
[setupvars.sh] OpenVINO environment initialized

#  update udev rules
sh /opt/intel/openvino/install_dependencies/install_NCS_udev_rules.sh
```

opencv support packages
```
sudo apt install libgtk-3.so.0
sudo apt install libcanberra-gtk*
```

## OpenVino python3  setup environment

![](/images/python-icon.png#logo)
## python3 virtualenv
```bash
mkdir project
mkdir project/venv
sudo apt install python3-pip
sudo pip3 install virtualenv
virtualenv -p python3 venv

source venv/bin/activate
(venv) pip install numpy
# create soft link between virtualenv to opencv so location
# - locate cv2.so location in /opt/intel
find  . -name "cv2*.so"
./openvino/python/python2.7/cv2.so
./openvino/python/python3.5/cv2.cpython-35m-arm-linux-gnueabihf.so

# from virtualenv lib folder create soft link
cd /home/pi/venv/lib/python3.5/site-packages
ln -s /opt/intel/openvino/python/python3.5/cv2.cpython-35m-arm-linux-gnueabihf.so cv2.so

# Add alias to activate / deactivate virtualenv
echo "alias ae='deactivate &> /dev/null; source ./venv/bin/activate'" >> ~/.bashrc
echo "alias de='deactivate'" >> ~/.bashrc

# activate ve
(venv)python
>>> import cv2
>>> print (cv2.__version__)
4.1.0-openvino
```

&nbsp;  
&nbsp;  
&nbsp;  


# Run test application
- Download face  detection models from intel pre-trained
```bash
#create models folder
mkdir ~/models
cd ~/models
#model
wget --no-check-certificate https://download.01.org/opencv/2019/open_model_zoo/R1/models_bin/face-detection-adas-0001/FP16/face-detection-adas-0001.bin
# xml topo
wget --no-check-certificate https://download.01.org/opencv/2019/open_model_zoo/R1/models_bin/face-detection-adas-0001/FP16/face-detection-adas-0001.xml
```

## python code
```python
import cv2 as cv
# Load the model.
#net = cv.dnn.readNetFromCaffe("~/models/face-detection-adas-0001.prototxt","~/models/face-detection-adas-0001.xml")
net = cv.dnn.readNet('/home/pi/models/face-detection-adas-0001.xml', '/home/pi/models/face-detection-adas-0001.bin')
# Specify target device.
net.setPreferableTarget(cv.dnn.DNN_TARGET_MYRIAD)
# Read an image.
frame = cv.imread("/home/pi/images/face.jpeg")
# Prepare input blob and perform an inference.
blob = cv.dnn.blobFromImage(frame, size=(672, 384), ddepth=cv.CV_8U)
net.setInput(blob)
out = net.forward()
# Draw detected faces on the frame.
for detection in out.reshape(-1, 7):
    confidence = float(detection[2])
    xmin = int(detection[3] * frame.shape[1])
    ymin = int(detection[4] * frame.shape[0])
    xmax = int(detection[5] * frame.shape[1])
    ymax = int(detection[6] * frame.shape[0])
    if confidence > 0.5:
        cv.rectangle(frame, (xmin, ymin), (xmax, ymax), color=(0, 255, 0),thickness=8)
# Save the frame to an image file.
cv.imwrite('out.png', frame)
```

## Result (in.jpeg ==> out.png)

![](/images/face_detaction.jpg)


# References
- [OpenVINO™ toolkit - RaspberryPI + Movidius NCS](https://www.youtube.com/watch?v=PNmH_ugW6Zw)
- [Install OpenVINO™ toolkit for Raspbian* OS](https://docs.openvinotoolkit.org/latest/_docs_install_guides_installing_openvino_raspbian.html)
- [OpenVINO, OpenCV, and Movidius NCS on the Raspberry Pi](https://www.pyimagesearch.com/2019/04/08/openvino-opencv-and-movidius-ncs-on-the-raspberry-pi/)
- [Python example](https://github.com/leswright1977/RPi3_NCS2)