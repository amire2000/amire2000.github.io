---
layout: post
title: Linux video utils and cheat sheet
categories: video
tags: [v4l]
public: true
image: film-editing.png
description: video utils and command line to run check and inspect video and images
---

# v4l2-ctl
- list device
  
```
v4l2-ctl --list-device
```

- check device support formats

```
v4l2-ctl --list-formats-ext --device=/dev/vide1
```