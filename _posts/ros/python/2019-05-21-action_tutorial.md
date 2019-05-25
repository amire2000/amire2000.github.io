---
layout: post
title: ROS Action 101
categories: ros
tags: [rospy, tutorial, action server, action client, action msg]
image: action.png
description: ROS Actionlib tutorial for beginners, create and compile action msg, Define action server and use it by implement action client
public: true
---
# Content
- Action
- Demo project
- [Action messages](#action-message)
- [Action Server](#action-server)
- [Action client](#action-client)


# Action
- Action are asynchronous RPC
- ROS action are the best way to implement interface time-extends operations
- Action are defined by there message type
  - goal (request)
  - result (response)
  - feedback


![Action interface](/images/2019-05-22-14-06-47.png)

- action server: ROS node that advertise an action so other nodes can call it.
- action client: ROS node that "send" goal requested to the action server
&nbsp;  
&nbsp;  
# Demo project
Implement Action service include message, ActionServer, ActionClient
- Action service export counter that count until requested `GOAL`
- return feedback every 1s, 
- goal response with status message 


## Project structure
```
├── action
│   └── Counter.action
├── CMakeLists.txt
├── package.xml
└── scripts
    ├── counter_action_server.py
    └── counter_action_client.py
```

&nbsp;  
&nbsp;  
# Action message
- Create `Counter.action` message file under `action` folder
- Add `message_generation` and `message_runtime` to `package.xml`
- Add `add_action_files` section and it's dependencies to `CMakeLists.txt`
- build `catkin build py_tutorial` (catkin-tools)

## action message file
Counter.action
```bash
#goal
uint32 num_count
---
#result
string result_message
---
#feedback
uint32 count_elapsed
```

## package.xml
- Add `message_generation` and `message_runtime`
```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>

<build_depend>actionlib_msgs</build_depend>
<exec_depend>actionlib_msgs</exec_depend>
```

## CMakeLists.txt
- Add `message_generation` to `find_package`
- Add `message_runtime` to `catkin_package`
- Add `add_action_files` section
- Add `generate_messages` section

```bash
cmake_minimum_required(VERSION 2.8.3)
project(py_tutorial)

#find_package
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  rospy
  message_generation
)

#add_action_files
## Generate actions in the 'action' folder
add_action_files(
  FILES
  WashTheDishes.action
)

## generate_messages
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
  DEPENDS message_runtime
)
```

## build
```
catkin build py_tutorial
```

build msg files 
> Create msg folder

```bash
roscd py_tutorial
rosrun actionlib_msgs genaction.py -o msg action/Counter.action
```

## action messages
- Check and understand build result
```bash
rosmsg list | grep Counter
#result
# Messages with  Action used by actionlib internally
py_tutorial/CounterAction
py_tutorial/CounterActionFeedback
py_tutorial/CounterActionGoal
py_tutorial/CounterActionResult
# Message used by user application
py_tutorial/CounterFeedback
py_tutorial/CounterGoal
py_tutorial/CounterResult
```

- Show message
```bash
#CounterFeedback
rosmsg show py_tutorial/CounterFeedback
uint32 count_elapsed

#CounterGoal
rosmsg show py_tutorial/CounterGoal
uint32 num_count

#CounterResult
rosmsg show py_tutorial/CounterResult
string result_message
```
&nbsp;  
&nbsp;  
# Action server
- SimpleServer code
- Action server topics
- Action server test

## Server code
- Action server
- Action server code
- Run
- Action server topics

### Action server
![action server states](/images/2019-05-24-01-36-41.png)


### Action server code
implement action server using `SimpleActionServer`

```python
#! /usr/bin/env python
import rospy
import actionlib
from py_tutorial.msg import CounterFeedback, CounterResult, CounterAction

class CounterActionServer(object):
    # create messages that are used to publish feedback/result
    _feedback = CounterFeedback()
    _result = CounterResult()

    def __init__(self, name):
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name, 
            CounterAction, 
            execute_cb=self.execute_cb, 
            auto_start = False)

        # Start the action server.
        self._as.start()
        rospy.loginfo("Action server started...")

    def execute_cb(self, goal):
        counter_delay_value = 1.0
                # Variable to decide the final state of the action server.
        success = True

        rospy.loginfo("action server is counting up to {} with {}s".format(
            goal.num_count, 
            counter_delay_value))
        rate = rospy.Rate(1)

        # start executing the action
        for counter_idx in range(0, goal.num_count):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            # publish the feedback
            self._feedback.count_elapsed = counter_idx
            self._as.publish_feedback(self._feedback)
            rate.sleep()

        if success:
            self._result.result_message = "Successfully completed counting."
            rospy.loginfo("Succeeded run action")
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    # Initialize a ROS node for this action server.
    rospy.init_node('counter_ac')
    server = CounterActionServer(rospy.get_name())
    rospy.spin()
```



## Sever test (Run)
- Terminal1(core)
```
roscore
```

- Termianl2(action server)
```
rosrun py_tutorial counter_action_server.py 
[INFO] [1558537494.706497]: Action server started...
[INFO] [1558537494.753959]: action server is counting up to 5 with 1.0s
```

- Terminal3 (subscribe to feedback topic)
```
rostopic echo /counter_ac/feedback
```

- Terminal4 (pub into goal)
  - TAB TAB to complete message
```
rostopic pub /counter_ac/goal py_tutorial/CounterActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  num_count: 5" 
```

## Server topics
```bash
rostopic list
/counter_ac/cancel
/counter_ac/feedback
/counter_ac/goal
/counter_ac/result
/counter_ac/status
```

## Notes
> SimpleActionServer/Client can execute **ONE** goal at a time  

> New Goal to active server pre-empt an active goal



# Action client
- Action client
- SimpleClient code example
- Run

## Action client
![ros action client states](/images/2019-05-24-01-30-35.png) 

## SimpleClient code example
- Using `SimpleActionClient` to implement ROS action client
- action client `init` take the server name  to connect to and the message type
> SimpleActionServer/Client can execute **ONE** goal at a time  


**counter_action_client.py**
```python
#! /usr/bin/env python
import rospy
import actionlib

from py_tutorial.msg import CounterAction, CounterGoal, CounterResult

# Without thread locking
done_flag = False

def feedback_handler(msg):
    rospy.loginfo("feedback: {}".format(msg))

def active_handler():
    rospy.loginfo("active: {}")

def donecb_handler(state, result):
    rospy.loginfo("state: {}".format(state))
    rospy.loginfo("result: {}".format(result))
    
    global done_flag
    done_flag = True

def counter_client():
    client = actionlib.SimpleActionClient("/counter_ac", CounterAction)
    # client.feedback_cb = feedback_handler
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()

    num_counts = 5
    goal = CounterGoal(num_counts)
    client.send_goal(goal, 
        done_cb=donecb_handler,
        active_cb=active_handler,
        feedback_cb=feedback_handler)
    rospy.loginfo("Goal has been sent to the action server.")
    
    while not done_flag:
        rospy.loginfo("Do other when goal are processed by the server")
        rospy.sleep(1.5)

    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('counter_client')
        result = counter_client()
        rospy.loginfo(result.result_message)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")

```


## run
- Terminal1 (core)
```bash
roscore
```

- Terminal2 (server)
```bash
rosrun py_tutorial counter_action_server.py
```

- Terminal3 (client)
```bash
rosrun py_tutorial counter_action_client.py
# Client output
[INFO] : Waiting for action server to come up...
[INFO] : Goal has been sent to the action server.
[INFO] : Do other when goal are processed by the server
[INFO] : active: {}
[INFO] : feedback: count_elapsed: 0
[INFO] : feedback: count_elapsed: 1
[INFO] : Do other when goal are processed by the server
[INFO] : feedback: count_elapsed: 2
[INFO] : Do other when goal are processed by the server
[INFO] : feedback: count_elapsed: 3
[INFO] : feedback: count_elapsed: 4
[INFO] : Do other when goal are processed by the server
[INFO] : state: 3
[INFO] : result: result_message: "Successfully completed counting."
[INFO] : Successfully completed counting.
```
# Reference
- [ROS catkin CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt#Messages.2C_Services.2C_and_Action_Targets)
- [ROS Actions client server communication](https://ocw.tudelft.nl/course-lectures/1-4-1-ros-actions-client-server-communication/)
- [ROS org actionlib](http://wiki.ros.org/actionlib)