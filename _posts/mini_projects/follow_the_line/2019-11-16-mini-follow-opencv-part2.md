---
layout: post
title: Follow the line - part 2
categories: [mini, line]
tags: [opencv, gazebo, python, jupyter]
public: true
description: Using python and OpenCV to find the line
image: follow_line.jpeg
---

LAB2 - Line following
- Using 
  - OpenCV
  - ROS Twist topic

# OpenCV
- Find line center using contours and center of mass
- Implement opencv algorithm using python and jupyter notebook for easy of use

## Get one image
- Tack image from gazebo for vision analysis
  
  
- gazebo save images in `~/.gazebo/pictures`

## Find the line
- Using jupyter notebook to implement find the line algorithm

- Step 1
  - load the image into notebook

```python cell
from matplotlib import pyplot as plt
import cv2

img=cv2.imread("~/pictures/line.jpg")
rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
plt.imshow(rgb_img)
plt.show()
```

![](/images/2019-11-16-19-33-02.png)

- Step 2
  - crop
  
```python
crop_img = img[300:600, 500:1200]
```

- Step 3
  - convert to gray
  - apply filter
  - apply threshold

```python
gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray,(5,5),0)
ret,thresh = cv2.threshold(blur,100,200,cv2.THRESH_BINARY)
```

- Step 4
  - Find contours
  - find center off mass

```python
contours,hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
if len(contours) > 0:
    c = max(contours, key=cv2.contourArea)
    M = cv2.moments(c)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    print(cx, cy)
    cv2.circle(crop_img, (cx, cy), 10, (255, 0, 0), 8)
    cv2.drawContours(crop_img, contours, -1, (0,255,0), 5)
    plt.imshow(crop_img)
    plt.show()
```

![](/images/2019-11-16-19-41-57.png)

# ROS Node
- Create cart_follow package
- Implement CvBridge node
- Implement OpenCV logic from above
- Send `Twist` message

## package folders

### node template
```python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge

TOPIC = "/cart/camera1/image_raw"
class Follower(object):
    def __init__(self):
        self._bridge = cv_bridge.CvBridge()
        
        sub = rospy.Subscriber(TOPIC, Image, self.image_callback)

    def image_callback(self, msg):
        image = self._bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        cv2.imshow("window", image)
        cv2.waitKey(3)


def on_shutdown():
    rospy.loginfo("shutdown")
    cv2.destroyAllWindows()

def main():
    rospy.init_node("follower")
    rospy.loginfo("start node")
    rospy.on_shutdown(on_shutdown)
    while not rospy.is_shutdown():
        follower = Follower()
        rospy.spin()

if __name__ == "__main__":
    main()
```

### Run node
> Don't forget to set executable permission

```bash
rosrun cart_follower follow.py
```