---
layout: post
title: Turtlesim goto goal
categories: [mini, turtlesim]
tags: [ros]
image: turtle_twist.jpg
public: True
description: Move turtle to goal , using Euclidean Distance and atan2 function, send Twist msg and subscribe to pose callback
---
<style>
img[src*='#size1'] {
    width: 400px;
    height: 200px;
}
</style>
## Euclidean Distance
Distance between to points in straight line

![](/images/2019-12-07-08-37-10.png#size1)

### 2d distance


&nbsp;  
&nbsp;  
## atan2

$$
\alpha = atan2(\frac{y}{x})
$$

![](/images/2019-12-07-08-57-58.png#size1)

&nbsp;  
&nbsp;  
# Goto
- Move turtle to goal point
  - Send `Twist` msg
  - Calc distance between turtle and goal using `Euclidean Distance`
  - Using distance to create linear velocity command (m/s)
  - Calc turtle heading to create angular velocity command (rad/sec)

```python
#!/usr/bin/env python
import rospy
import sys
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from math import pow, atan2, sqrt

class Goto():

    def __init__(self):
        self.sub = rospy.Subscriber("/turtle1/pose", Pose, self.pose_cb)
        self.publisher = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=1)
        self.pose = Pose()
        self.rate = rospy.Rate(1)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.1):
        return constant * self.euclidean_distance(goal_pose)


    def angular_vel(self, goal_pose, constant=0.5):
        angle = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
        return constant * (angle - self.pose.theta)

    def pose_cb(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 2)
        self.pose.y = round(self.pose.y, 2)
        

    def move2goal(self, pos_x=1, pos_y=9):
        msg = Twist()
        goal_pose = Pose()
        goal_pose.x = pos_x
        goal_pose.y = pos_y
        distance_tolerance = 0.1
        while True:
            d = self.euclidean_distance(goal_pose)
            if d <= distance_tolerance:
                break

            msg.linear.x = self.linear_vel(goal_pose)
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.publisher.publish(msg)
            if rospy.is_shutdown():
                rospy.logwarn('ctl-c shutdown')
                break
            self.rate.sleep()


def main(args):
    rospy.init_node('simple_goto', anonymous=True)
    obj = Goto()
    obj.move2goal()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")


if __name__ == '__main__':
    main(sys.argv)

```

![](/images/2019-12-07-09-22-11.png)