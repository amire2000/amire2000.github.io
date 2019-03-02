

```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

## ROS Service
```
rosservice call /mavros/cmd/arming "value: true"
```
## Reference
- [Install melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Mavros install](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)