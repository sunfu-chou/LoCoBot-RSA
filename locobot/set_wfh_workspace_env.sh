#! /bin/bash

# load pyrobot env
load_pyrobot_env
# source WFH workspace and set_rospkg_path
source ~/WFH_locobot/set_ip.sh $1 $2
#source ROS/catkin_ws/devel/setup.bash 
#source  ROS/catkin_ws/devel_isolated/setup.bash
source ~/WFH_locobot/set_rospackage_path.sh
echo "finish setup WFH env"
