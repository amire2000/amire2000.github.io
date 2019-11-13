---
layout: post
title: Gazebo image viewer
categories: gazebo
tags: [standalone]
public: true
---

![](/images/2019-11-13-09-15-21.png)

```bash
gz topic --list
/gazebo/default/atmosphere
/gazebo/default/cart/joint_cmd
/gazebo/default/cart/link_chassis/camera_sensor/cmd
/gazebo/default/cart/link_chassis/camera_sensor/image
/gazebo/default/cart/link_chassis/wrench
/gazebo/default/cart/link_left_wheel/wrench
/gazebo/default/cart/link_right_wheel/wrench
/gazebo/default/diagnostics
/gazebo/default/factory

```

```
gz topic --info /gazebo/default/cart/link_chassis/camera_sensor/image
Type: gazebo.msgs.ImageStamped

Publishers:
	192.168.2.253:42799

Subscribers:

```

```cpp


#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define IMAGE_TOPIC "~/cart/link_chassis/camera_sensor/image"

void cb(ConstImageStampedPtr &msg)
{
    int width;
    int height;
    char *data;
    std::cout << "cb\n";
    width = (int) msg->image().width();
    height = (int) msg->image().height();
    //+1 for null terminate
    data = new char[msg->image().data().length() + 1];

    memcpy(data, msg->image().data().c_str(), msg->image().data().length());
    //gazebo output rgb data
    //PixelFormat.R8G8B8
    cv::Mat image(height, width, CV_8UC3, data);

    cv::imshow("camera", image);
    cv::waitKey(1);
    delete data;  // DO NOT FORGET TO DELETE THIS, 
                  // ELSE GAZEBO WILL TAKE ALL YOUR MEMORY
}
 

int main(int argc, char **argv)
{
    std::cout << "hello \n";
    gazebo::client::setup(argc, argv);
    //creat node
    gazebo::transport::NodePtr node(
        new gazebo::transport::Node());
    node->Init();
    std::cout << IMAGE_TOPIC << std::endl;
    //Listener
    gazebo::transport::SubscriberPtr sub =
         node->Subscribe(IMAGE_TOPIC, cb);
    while (true){
        gazebo::common::Time::MSleep(10);
    }
    gazebo::client::shutdown();
    return 0;
}
```


## meson
```python
gz = dependency('gazebo')
cv = dependency('opencv')

install_dir = meson.source_root() + '/bin'

executable('camera_v', 'image_viewer/main.cpp',
        dependencies : [gz, cv],
        install: true,
        install_dir: [install_dir])
```