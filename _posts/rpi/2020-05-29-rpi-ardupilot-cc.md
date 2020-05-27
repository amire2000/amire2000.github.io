---
layout: post
title: Prepared RPi as ardupilot companion computer
categories: rpi
tags: [mavproxy]
image: 
description: 
public: true
---

# LAB
- RPi 4
- Ubuntu 18.04 (64bit)

## setup
- Install mavproxy
- Add Startup script

# Install and setup
```
sudo apt update
sudo apt install python3-pip
sudo apt-get install libxml2-dev libxslt-dev python-dev
sudo apt remove modemmanager
#gstreamer
sudo apt-get install gstreamer1.0-tools
sudo apt-get install gstreamer1.0-plugins-good
sudo apt-get install gstreamer1.0-plugins-bad
sudo apt-get install gstreamer1.0-libav
sudo apt-get install libglib2.0-dev
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

sudo apt install v4l-utils
sudo apt install ffmpeg
sudo apt install tmux

```
## install mavproxy
```
pip3 install serial
pip3 install mavproxy
echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
```

## Setup
- add user to `dialout`
- remove `modemmanager`

```
sudo adduser <username> dialout
```

# Startup script
- Add script to folder `/etc/systemd/system`
    - script name `myservice.service`
- Add start.sh script to user `ubuntu` home directory
  - this script run as root by the service
- Add script run with user `ubuntu` as owner


### myservice.service

```
[Service]
ExecStart=/home/ubuntu/start.sh
[Install]
WantedBy=default.target
```

### start.sh
```
#!/usr/bin/env bash
echo "ran at $(date)!" > /tmp/start

cd /home/ubuntu
sudo -u ubuntu /home/ubuntu/session.sh
```

### session.sh
```
```

&nbsp;  
&nbsp;  
### enable and check
```bash
sudo systemctl enable myservice
# check
sudo systemctl start myservice
sudo systemctl status myservice
```

&nbsp;  
&nbsp;  
&nbsp;  

# Reference
- [RTSP streaming from Raspberry PI](https://gist.github.com/neilyoung/8216c6cf0c7b69e25a152fde1c022a5d)
- [How to Enable /etc/rc.local with Systemd](https://www.linuxbabe.com/linux-server/how-to-enable-etcrc-local-with-systemd)
