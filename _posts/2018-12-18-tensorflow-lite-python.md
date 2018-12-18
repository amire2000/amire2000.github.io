---
layout: post
title: Tensorflow lite python raspberry pi
categories: tensorflow
tags: [tensorflow, raspberry pi]
---
|![](/images/2018-12-18-23-44-59.png)     | ![](/images/2018-12-18-23-51-53.png)    | ![](/images/2018-12-18-23-45-36.png)    |
| --- | --- | --- |



## Raspbian Image
- Download Rasbian 9.0 desktop
- Flash image
- Add ssh empty file  to boot partition to allow ssh connection

## Install Tensorflow
- Prepared
- Create virtualenv
- Download and install whl

[
Install TensorFlow with pip](https://www.tensorflow.org/install/pip)

```
$ sudo apt update
$ sudo apt install python3-dev $ python3-pip
$ sudo pip3 install -U virtualenv
```
- Create and activate ve
```
$ virtualenv --system-site-packages -p python3 ./tf
$ source ./tf/bin/activate
```

- Install tensorflow
> Don't install from office  site
> Install from this [url](https://github.com/PINTO0309/Tensorflow-bin) solved #15062, #21574, #21855, #23082

```bash
$ sudo apt-get install libatlas-base-dev
$ sudo apt-get install python-pip python3-pip python-scipy libhdf5-dev
$ sudo apt-get install -y openmpi-bin libopenmpi-dev
$ sudo pip2 uninstall tensorflow
$ wget -O tensorflow-1.11.0-cp27-cp27mu-linux_armv7l.whl https://github.com/PINTO0309/Tensorflow-bin/raw/master/tensorflow-1.11.0-cp27-cp27mu-linux_armv7l_jemalloc.whl
$ sudo pip2 install tensorflow-1.11.0-cp27-cp27mu-linux_armv7l.whl
```

### check
```bash
(tf)$ python
...
>>> import tensorflow
>>> tensorflow.__version__
```

## Convert 
- Open `object detection_ssd_coco.md` file from `tensorflow/tensorflow/contrib/lite/example/python/`
- Download model (untar into `/tmp` folder)
- Convert grap to tflite with bazel
- Install bazel (host) from deb file [0.19.1](https://github.com/bazelbuild/bazel/releases/tag/0.19.1) 
  - other version run with errors
- Clone repo in host `git clone https://github.com/freedomtan/tensorflow.git`
  - checkout `deeplab_tflite_python` branch
```
 $ git branch
* deeplab_tflite_python
  master
```
- Run convert from root repo folder (`WORKSPACE` file)
```
bazel run -c opt   //tensorflow/contrib/lite/toco:toco -- --input_file=/tmp/ssd_mobilenet_v1_coco_2017_11_17/frozen_inference_graph.pb  --output_file=/tmp/ssd_mobilenet_v1_coco.tflite --inference_type=FLOAT --input_shape=1,300,300,3 --input_array=Preprocessor/sub --output_arrays=concat,concat_1
```
> Replace --config=opt with -c opt
- I Got some `ccache error` after some google search i export gcc and g++ location
  ```
    export CC="/usr/bin/gcc"
    export CXX="/usr/bin/g++"
  ```
- Copy label and priors files
```
$ cp ${TF_ROOT}/tensorflow/contrib/lite/examples/android/assets/box_priors.txt /tmp
$ cp ${TF_ROOT}/tensorflow/contrib/lite/examples/android/assets/coco_labels_list.txt  /tmp
```
- Download test image `image2.jpg` from `tensorflow/model/research/object_detection/test_images`
- copy/scp all files to rpi
  - model: `ssd_mobilenet_v1_coco.tflite`
  - box_priors.txt
  - coco_labels_list.txt
  - image2.jpg
  - source code: `object_detection.py`
  - run (from tf virtualenv)
  ```
  (tf)$ python tensorflow/contrib/lite/examples/python/object_detection.py --graph /tmp/ssd_mobilenet_v1_coco.tflite   --image /tmp/image2.jpg  --show_image True

  ```
> PreInstall Note:
> 
> `sudo pip3 install pillow lxml matplotlib cython`
> 
> `sudo apt-get install python3-tk`


### Run (from pi home folder and ve active)
```bash
(tf)$ python object_detection.py --graph ssd_mobilenet_v1_coco.tflite   --image image2.jpg  --show_image True
```
![](/images/2018-12-18-23-48-52.png)
## Reference
- [Prebuild for raspberry](https://github.com/PINTO0309/Tensorflow-bin)
- [freedomdan/tensorflow (deeplab_tflite_python branch)](https://github.com/freedomtan/tensorflow/tree/deeplab_tflite_python)
