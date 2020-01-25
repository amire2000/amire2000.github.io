---
layout: post
title: Write v4l2
categories: video
tags: [v4l2loopback]
---

```
//  g++ m.cpp -o app `pkg-config --cflags --libs opencv`

#include "opencv2/opencv.hpp"
#include <iostream>
#include <unistd.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
// #include <sys/ioctl.h>

using namespace std;
using namespace cv;

int fdwr;

# define FRAME_WIDTH  640
# define FRAME_HEIGHT 480
int imageSize = FRAME_WIDTH * FRAME_HEIGHT * 3;

cv::Size yuv_img_sz = cv::Size(FRAME_WIDTH, FRAME_HEIGHT);
//YUV buffer

__u8 *buffer = (__u8 *)malloc(sizeof(__u8) * imageSize);


void convert(Mat yuv_img)
{
    size_t nx;
    size_t ny;
    int i = 0;
    __u8 r0, g0, b0, r1, g1, b1;
    __u8 y0, y1, u, v;
    int ncol;
    uchar *pcol;
    cv::cvtColor(yuv_img, yuv_img, CV_BGR2RGB);
    ncol = yuv_img.cols * yuv_img.channels();
    cout << yuv_img.rows << endl;
    for (ny = 0; ny < yuv_img.rows; ny++)
    {
        pcol = yuv_img.ptr<uchar>(ny);
        for (nx = 0; nx < ncol; nx = nx + 6)
        {
            r0 = (__u8)pcol[nx];
            g0 = (__u8)pcol[nx + 1];
            b0 = (__u8)pcol[nx + 2];
            r1 = (__u8)pcol[nx + 3];
            g1 = (__u8)pcol[nx + 4];
            b1 = (__u8)pcol[nx + 5];

            y0 = (__u8)(0.299 * r0 + 0.587 * g0 + 0.114 * b0);
            y1 = (__u8)(0.299 * r1 + 0.587 * g1 + 0.114 * b1);
            u = (__u8)(0.436 * (b0 - y0) / (1 - 0.114) + 128);
            v = (__u8)(0.615 * (r0 - y0) / (1 - 0.299) + 128);

            buffer[i++] = y0;
            buffer[i++] = v;
            buffer[i++] = y1;
            buffer[i++] = u;
        }
        if (i > imageSize)
        {
            cout << "error";
        }
    }
    cout << "as";
    write(fdwr, buffer, imageSize);
}

int main()
{
    fdwr = open("/dev/video2", O_RDWR);
    memset(buffer, 0, imageSize);
    VideoCapture cap("/dev/video1");

    // Check if camera opened successfully
    if (!cap.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    while (1)
    {

        Mat frame;
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
            break;
        convert(frame);
        imshow("Frame", frame);
        char c = (char)waitKey(25);
        if (c == 27)
            break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
```