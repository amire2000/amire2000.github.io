---
layout: post
title: Turtlesim Twist and Pose
categories: [mini, turtlesim]
tags: [ros]
image: turtle_twist.jpg
public: True
description: Learn ROS basic with turtlesim, launch nodes, publish and subscribers using Twist and Pose messages
---


# Subscriber
- Basic subscriber implement as python class

- Run `rostopic list` 
- Run `rostopic info` to check topic msg
- Run `rosmsg info` to get message fields/ properties

![](/images/2019-12-04-08-50-36.png)

```python
#!/usr/bin/env python
import rospy
import sys
from turtlesim.msg import Pose

class simple_sub():

  def __init__(self):
    self.sub = rospy.Subscriber("/turtle1/pose", Pose, self.callback)

  def callback(self, data):
    rospy.loginfo("x: {}, y: {}".format(data.x, data.y))



def main(args):
  sub = simple_sub()
  rospy.init_node('simple_class', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.logerr("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
```

# Publisher
- Basic publlisher implement as python class
- Using `cmd_vel` as demo topic
- Publishe linear `Twist` msg

![](/images/2019-12-04-09-23-13.png)

```python
#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist

class simple_pub():
    def __init__(self):
        self.publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

    def publish(self, linear_x):
        msg = Twist()
        msg.linear.x = linear_x
        self.publisher.publish(msg)

if __name__ == "__main__":
    rospy.init_node('simple_pub', anonymous=True)
    rate = rospy.Rate(0.5)
    pub = simple_pub()
    
    while not rospy.is_shutdown():
        rospy.loginfo("start")
        pub.publish(1)
        rate.sleep()
```
