#! /bin/bash

# load pyrobot env
load_pyrobot_env
# source WFH workspace and set_rospkg_path
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

#source ROS/catkin_ws/devel/setup.bash 
#source  ROS/catkin_ws/devel_isolated/setup.bash
source ~/WFH_locobot/set_rospackage_path.sh
echo "finish setup WFH env"
