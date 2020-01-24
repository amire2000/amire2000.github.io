---
layout: post
title: HTML 5 video streaming
categories: web
tags: [video, gstreamer]
description: Using html5 video tag to show html5 streaming
image: HTML5_video.png
public: true
---
HTML5 support three video formats: MP4, WebM, and Ogg.

## Test browser support
```
http://html5test.com/
```

![](/images/2020-01-23-06-17-27.png)

```html
<!DOCTYPE html>
<html>
        <head>
                <meta http-equiv="content-type" content="text/html; charset=utf-8">
                <title>gst-stream</title>
        </head>
        <body>
                <video width=320 height=240 autoplay>
                        <source src="http://localhost:8080">
                </video>
        </body>
</html>
```

```bash
gst-launch-1.0 \
    videotestsrc \
    ! videoconvert \
    ! videoscale \
    ! video/x-raw,width=320,height=240 \
    ! clockoverlay shaded-background=true font-desc="Sans 38" \
    ! theoraenc \
    ! oggmux \
    ! tcpserversink host=127.0.0.1 port=8080
```

-  Theora is a royalty-free video codec based on the VP3 codec.
-  Ogg is a free, open container format 