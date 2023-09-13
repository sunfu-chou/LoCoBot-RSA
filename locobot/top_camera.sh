#!/bin/bash

source ~/WFH_locobot/environment.sh
source ~/WFH_locobot/set_ip.sh $1 $2
rosservice call /calibration
#rostopic pub /tilt/command std_msgs/Float64 "data: 0.8"
#rostopic pub /pan/command std_msgs/Float64 "data: 0.0"
roslaunch realsense2_camera side_camera_1.launch
