---
layout: post
title: ROS Python UnitTest
categories: ros
tags: [tdd, unittest, rospy]
public: true
description: ROS UnitTest using python and rostest, 
image: unittest.png
---

- unit test
- integration test


# Content
- Install
- Rostest
- Usage

&nbsp;  
&nbsp;  
# Install
```
sudo apt install ros-melodic-rostest
```
&nbsp;  
&nbsp;  
# Rostest
Integration test suite based on roslaunch that is compatible with xUnit frameworks.

```
package
|
├── launch
│   └── test_code.test
├── package.xml
├── CMakeLists.txt
├── src
└── test
    └── test_code.py
```

## test_code.test (launch)

```xml
<launch>
  <test test-name="xxx_name" pkg="py_tutorial" type="test_code.py" />
</launch>
```

## test_code.py (test code)

> Note: Add execute permission to test_code.py

```python
#! /usr/bin/env python
import unittest
import rostest
class MyTestCase(unittest.TestCase):

    def test_whatever(self):
        self.assertEqual(1,2, "fail")


if __name__ == "__main__":
    #rostest.rosrun(<package>, <module>, <class>)
    rostest.rosrun("py_tutorial", "test_code", MyTestCase)
```
&nbsp;  
&nbsp;  
# usage
## command line
- launch extension

```
rostest py_tutorial test_code.test
```

## catkin (CMakeLists.txt)
- Add macros
  - find_package
  - add_rostest

```cmake
find_package(rostest REQUIRED)
add_rostest(launch/test_code.test)
```

### Execute 
> using catkin tools

```bash
# Run test for all catkin packages
catkin run_tests

# Run test for specific package
catkin build <package> --catkin-make-args run_tests

# View test results
catkin_test_results build/<package name>
```

# Reference
- [Ros Testing Tutorial](https://www.slideshare.net/vgonpa/ros-testing-tutorial)
- [055 – Four different ways of testing your ROS python code](http://www.theconstructsim.com/ros-5-mins-055-four-different-ways-testing-ros-python-code/)
- [catkin tools , build with tests](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html#building-and-running-tests)