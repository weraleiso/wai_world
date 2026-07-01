#!/usr/bin/env bash

RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
BLUE='\033[34m'
MAGENTA='\033[35m'
CYAN='\033[36m'
WHITE='\033[37m'

echo -e "${WHITE}==============================${WHITE}"
echo -e "${CYAN}WAI WORLD - MAINTENANCE SCRIPT${WHITE}"
echo -e "${CYAN}*** AUDIENCE Bringup WAI World *** ${WHITE}"
echo -e "${WHITE}==============================${WHITE}"
echo -e "${WHITE}...${WHITE}"
echo -e "${WHITE}The ${YELLOW}whole audience${WHITE} is going to ${YELLOW}bringup WAI World${WHITE}!"
echo -e "${WHITE}...${WHITE}"
echo -e "${RED}Press any key to confirm...!${WHITE}"
read

for oa_ip in $(seq 1 10);
do
    if ping -c 1 10.0.0.$oa_ip &> /dev/null
    then
        echo -e "${YELLOW}Starting${WHITE} to bringup (${YELLOW}WAI World${WHITE}) with listener with IP 10.0.0.$oa_ip...${WHITE}!"
        sshpass -v -p ias ssh -t ias@10.0.0.$oa_ip "export DISPLAY=:0; source /opt/ros/noetic/setup.bash; source /home/ias/catkin_ws/devel/setup.bash; export ROS_IP=10.0.0.$oa_ip; export ROS_MASTER_URI=http://10.0.0.101:12412; export WAI_OA_AUDIENCE_ID=$[$oa_ip-1]; roslaunch wai_oa_gazebo wai_oa_audience_gazebo_spawn.launch" &
        sleep 5
        echo -e "${GREEN}Finished${WHITE} to bringup (${GREEN}WAI World${WHITE}) with listener with IP 10.0.0.$oa_ip...${WHITE}!"
    else
        echo -e "${WHITE}Listener with IP 10.0.0.$oa_ip is ${YELLOW}not reachable!${WHITE}!"
    fi
done

echo -e "${WHITE}...${WHITE}"
echo -e "${GREEN}FINISHED WITH MAINTENANCE!${WHITE}"

