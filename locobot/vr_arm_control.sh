#! /bin/bash
cd ~/WFH_locobot
source ~/WFH_locobot/set_wfh_workspace_env.sh

source ~/WFH_locobot/set_ip.sh $1 $2

rosrun oculusVR vrarm.py


