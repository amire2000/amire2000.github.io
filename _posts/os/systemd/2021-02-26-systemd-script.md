---
layout: post
title: Manage and control custom services with systemd
categories: os
tags: [startup, ubuntu]
public: true
description: Manage and control custom services with systemd
image: systemd-light.svg
---

# Demo
## Service Application
- Don't forget to `chmod`
```bash
#!/bin/bash
while true
do
    echo The current time is $(date)
    sleep 1
done
```

## Service file
- Place service file in `/etc/systemd/system` folder
- name the file with `.service` extension

my_service.service

```ini
[Service]
ExecStart=/home/user/tmp/my_service.sh
```

### run service and check
```bash
sudo systemctl start my_service
sudo systemctl status my_service
```

![](/images/2021-02-26-14-58-50.png)

> Default service output goto to `syslog`

```bash
tail /var/log/syslog
# or user 
journalctl -u my_service -f
#-u: select service to watch
#-f: 
```

# service file option and control
## unit options
| option  | description  |
|---|---|
| Description  |   |
| After  |   |
## Service options
| option  | description  |
|---|---|
| Restart  | always  |
| WorkingDirectory  |   |
| User  |   |
| Group  |   |
| Environment  |   |
| EnvironmentFile  |   |

## Install options
| option  | description  |
|---|---|
| WantedBy | always  |


# systemctl
```bash
# Start service on system bootup
systemctl enable <service>

# Stop running service on boot time
systemctl disable <service>

# reload definition after editing
systemctl reload-daemon

# Restart
systemctl <service>
```

# Demo: run mavproxy on startup
```ini
[Unit]
Description=Run Mavproxy by system as service Service
After=network.target

[Service]
Type=simple
User=user
WorkingDirectory
ExecStart
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

&nbsp;  
&nbsp;  
&nbsp;  
# Demo2 (python file)

- `my_service.py`
```python
#!/usr/bin/python3

import time

while True:
    print ("This is a test python file!")
    time.sleep(2)
```

## Create service file
`sudo vim /etc/systemd/system/my-py.service`

```
[Unit]
Description=Test Service
After=multi-user.target
Conflicts=getty@tty1.service

[Service]
Type=simple
ExecStart=/usr/bin/python3 /home/user/tmp/my_service.py
StandardInput=tty-force

[Install]
WantedBy=multi-user.target
```

##  Test

```bash
sudo systemctl daemon-reload
sudo systemctl enable my-py.service
sudo systemctl start my-py.service
sudo systemctl status my-py.service
```

![](/images/2021-02-28-00-33-21.png)