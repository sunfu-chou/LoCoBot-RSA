#! /bin/bash
cd ~/WFH_locobot
source ~/WFH_locobot/set_wfh_workspace_env.sh

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



rosrun oculusVR vrarm.py


