---
layout: post
title: video pixel format
categories: video
tags: [video, ]
public: true
description: Video Pixel format, RGB, YUV and other sampling
---

<style>
img[src*='#size1'] {
    width: 200;
    height: 200px;
}

img[src*='#size2'] {
    width: 300;
    height: 200px;
}
</style>

## YUV

- `Y` represent the brightness (luma)
- `U` chroma plane (`Cr`)
- `V` chroma plane (`Cb`)

### chroma planes
![](/images/2020-10-10-18-17-40.png#size1)
- from wikipedia

### subsampling
- Devid image to block
  - block height 2px
  - block width defined by the first number in the schema `4:4:4`
  - each block is 4*2 pixel
- brightness information separated from the color

![](/images/2020-10-10-18-33-45.png#size2)

- Chroma subsampling as it's name only apply to color channels

- The second number in the schema define no of sampling in upper line

![](/images/2020-10-10-18-36-33.png#size2)

- The third number in the schema define no of sampling in bottom line
  
![](/images/2020-10-10-18-37-25.png#size2)

<hr/>
#### 4:2:2

![](/images/2020-10-10-18-40-47.png#size2)

![](/images/2020-10-10-18-41-33.png#size2)

![](/images/2020-10-10-18-43-02.png#size2)
<hr/>
#### 4:2:0

![](/images/2020-10-10-21-44-08.png#size2)

- Zero: copy sampling from upper row to bottom

![](/images/2020-10-10-21-44-58.png#size2)

&nbsp;  
&nbsp;  
### packed, planar and semi-planar
- Packed (or interleaved)
- Planar (names often end with "p")
- Semi-planar (names often end with "sp")


#### Packed (or interleaved)
Packed means the components of Y, U, and V are interleaved, 

#### Planer
Planar means the components of Y, U, and V are respectively grouped together.
- I420: YYYYYYYY UU VV    =>YUV420P
- YV12: YYYYYYYY VV UU    =>YUV420P

#### Semi planar
Semi-planar means the components of Y are grouped together, and the components of U and V are interleaved

- NV12: YYYYYYYY UVUV     =>YUV420SP
- NV21: YYYYYYYY VUVU     =>YUV420SP

![](/images/2020-10-10-22-02-17.png)


&nbsp;  
&nbsp;  
&nbsp;  
### YUY2
- interleaved


&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [About YUV formats](https://gist.github.com/Jim-Bar/3cbba684a71d1a9d468a6711a6eddbeb)
- [Chroma subsampling](https://youtu.be/fkz2-JVaYDk?t=80)
- [Understanding YUV data formats](https://www.flir.eu/support-center/iis/machine-vision/knowledge-base/understanding-yuv-data-formats/)
- [Recommended 8-Bit YUV Formats for Video Rendering](https://docs.microsoft.com/en-us/windows/win32/medfound/recommended-8-bit-yuv-formats-for-video-rendering)