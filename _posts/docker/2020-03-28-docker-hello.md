---
layout: post
title: Docker hello
categories: docker
tags: []
public: true
---

# Share Volumes

- Mount local host tmp folder as gust `/root/tmp`
- `--rm` remove container on exit
- `-name` container name


```
 docker run -it --rm \
 -v ~/tmp:/root/tmp \
 --name cross \
 ubuntu:18.04
```

```bash
docker ps
#
CONTAINER ID        IMAGE               COMMAND             CREATED             STATUS              PORTS               NAMES
6abcefe9d678        ubuntu:18.04        "/bin/bash"         19 seconds ago      Up 19 seconds                           cross

```
# Dockerfile
```
FROM ubuntu:18.04
MAINTAINER ae@dev.com

RUN apt-get update && apt-get install -y openssh-server
RUN useradd -rm -d /home/user -s /bin/bash -g root -G sudo -u 1000 -p "$(openssl passwd -1 user)" user

EXPOSE 22

USER user
WORKDIR /home/user
CMD ["/bin/bash"]
```

## Run
```bash
# from Dockerfile folder run:
docker build -t cross:last .
```

# Network
```
docker run -it --rm \
 -v ~/tmp:/root/tmp \
 -p 2222:22 \
 --name cross \
 cross:latest
```