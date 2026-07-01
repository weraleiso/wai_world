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
echo -e "${CYAN}*** AUDIENCE Compile Package *** ${WHITE}"
echo -e "${WHITE}==============================${WHITE}"
echo -e "${WHITE}...${WHITE}"
echo -e "${WHITE}The ${YELLOW}whole audience${WHITE} is going to be provided with a ${YELLOW}new package${WHITE}!"
echo -e "${WHITE}...${WHITE}"
echo -e "${WHITE}Type in the package name:${WHITE}"
read world_pkg

for oa_ip in $(seq 1 10);
do
    if ping -c 1 10.0.0.$oa_ip &> /dev/null
    then
        echo -e "${YELLOW}Starting${WHITE} to maintain (${YELLOW}update package $world_pkg${WHITE}) listener with IP 10.0.0.$oa_ip...${WHITE}!"
        sshpass -v -p ias ssh ias@10.0.0.$oa_ip "source /opt/ros/noetic/setup.bash; rm -rf /home/ias/catkin_ws/src/wai_world/$world_pkg"
        sshpass -v -p ias ssh ias@10.0.0.$oa_ip 'sync; sync; sync;'
        sshpass -v -p ias scp -r /home/ias/catkin_ws/src/wai_world/$world_pkg ias@10.0.0.$oa_ip:/home/ias/catkin_ws/src/wai_world/$world_pkg
        sshpass -v -p ias ssh ias@10.0.0.$oa_ip "sync; sync; sync; source /opt/ros/noetic/setup.bash; source /home/ias/catkin_ws/devel/setup.bash; cd /home/ias/catkin_ws; catkin build; source /home/ias/catkin_ws/devel/setup.bash"
        echo -e "${GREEN}Finished${WHITE} to maintain (${GREEN}update package $world_pkg${WHITE}) listener with IP 10.0.0.$oa_ip...${WHITE}!"
    else
        echo -e "${WHITE}Listener with IP 10.0.0.$oa_ip is ${YELLOW}not reachable!${WHITE}!"
    fi
done

echo -e "${WHITE}...${WHITE}"
echo -e "${GREEN}FINISHED WITH MAINTENANCE!${WHITE}"

