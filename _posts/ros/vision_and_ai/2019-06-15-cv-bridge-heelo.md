

```python
#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


WIN_NAME = "cv"

class DemoBridge(object):
    def __init__(self):
        rospy.init_node("cv_demo")
        cv2.namedWindow(WIN_NAME)
        self._bridge = CvBridge()
        self._sub = rospy.Subscriber("/mysensor/camera/image_raw",
            Image,
            self.cb)

    def cb(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow(WIN_NAME, cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            rospy.logerr(e)
        
        
def main():
    obj = DemoBridge()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.DestroyAllWindows()

if __name__ == "__main__":
    main()
```