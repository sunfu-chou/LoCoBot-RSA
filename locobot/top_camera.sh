#!/bin/bash

if [ "$1" ]; then

    echo "ROS MASRER $1"

    export ROS_MASTER_URI=http://$1:11311

else

    echo "ROS MASRER 127.0.0.1"

    export ROS_MASTER_URI=http://127.0.0.1:11311

fi



if [ "$2" ]; then

    echo "ROS IP $2"

    export ROS_IP=$2

else

    echo "ROS IP 127.0.0.1"

    export ROS_IP=127.0.0.1

fi

source ~/WFH_locobot/environment.sh
rosservice call /calibration
#rostopic pub /tilt/command std_msgs/Float64 "data: 0.8"
#rostopic pub /pan/command std_msgs/Float64 "data: 0.0"
roslaunch realsense2_camera side_camera_1.launch
