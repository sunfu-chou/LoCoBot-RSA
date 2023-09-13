#!/usr/bin/env bash
#
# Typical usage: ./join.bash subt
#

if [ ! -z "$1" ]; then
    ROS_MASTER_URI=http://$1:11311
    echo "ROS_MASTER $1"
fi

if [ ! -z "$2" ]; then
    ROS_IP=$2
    echo "ROS_IP $2"
fi

BASH_OPTION=bash

IMG=jackychen777/locobot:NUC-vr \

xhost +
# containerid=$(docker ps -aqf "ancestor=${IMG}") && echo $containerid
# Get the first ID if existing more than one IDs
containerid=$(docker ps -aqf "ancestor=${IMG}" | awk 'NR==1{print $1}') && echo $containerid
docker exec -it \
    --privileged \
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e ROS_IP=$ROS_IP \
    -e DISPLAY=${DISPLAY} \
    -e LINES="$(tput lines)" \
    ${containerid} \
    $BASH_OPTION
xhost -
