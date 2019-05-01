---
layout: post
title: orb2-slam
categories: robotics
tags: [orb, SLAM, robotics]
---

- Pull docker `youyu/orb_slam2`
- Run docker with X11 support
```bash
sudo xhost +
docker run -it --rm \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix  \
youyu/orb_slam2:latest
```
- Install and commit
  - openssh
  - x11-apps `check gui`
- Add share volume with docker `-v host:gust`
  - 
- Add port mapping to ssh `-p host:gust` 

```bash
sudo xhost +
docker run -it --rm \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/user/projects/orb-slam/datasets:/root/datasets \
orb_slam:1
```


## Monocular
- Download dataset
```
cd ~/projects/datasets
wget http://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
tar -zxvf rgbd_dataset_freiburg1_xyz.tgz
rm -rf rgbd_dataset_freiburg1_xyz.tgz
```
- Docker Terminal
```
cd /opt/ORB_SLAM2/Examples/Monocular
./mono_tum ../../Vocabulary/ORBvoc.txt \
TUM1.yaml \
/root/datasets/rgbd_dataset_freiburg1_xyz
```

## EuRoC Dataset 
- From /opt/ORB_SLAM2
```
./Examples/Stereo/stereo_euroc \
Vocabulary/ORBvoc.txt \
Examples/Stereo/EuRoC.yaml \
/root/datasets/mav0/cam0/data \
/root/datasets/mav0/cam1/data \
Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```

## Reference
- [Cartographer SLAM for Non-GPS Navigation](http://ardupilot.org/dev/docs/ros-cartographer-slam.html)
- [Run orb_slam2 on ubuntu](http://www.voidcn.com/article/p-gzjqneuf-bgn.html)
- [jzijlmans/orb_slam2_mod](https://github.com/jzijlmans/orb_slam2_mod)

