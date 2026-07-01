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
echo -e "${CYAN}*** AUDIENCE Copy RViz Config *** ${WHITE}"
echo -e "${WHITE}==============================${WHITE}"
echo -e "${WHITE}...${WHITE}"
echo -e "${WHITE}The ${YELLOW}whole audience${WHITE} is going to be ${YELLOW}provided with a new RViz configuration${WHITE}!"
echo -e "${WHITE}...${WHITE}"
echo -e "${WHITE}Press any key to confirm...!${WHITE}"
read

echo -e "${YELLOW}Starting${WHITE} to maintain (${YELLOW}RViz configuration${WHITE}) listener with IP 10.0.0.$oa_ip...${WHITE}!"
parallel-scp -r -h oa_listener_hosts.txt -l ias -p 10 -v -A /home/ias/catkin_ws/src/wai_world/wai_world_bringup/rviz /home/ias/catkin_ws/src/wai_world/wai_world_bringup
sync; sync; sync
echo -e "${GREEN}Finished${WHITE} to maintain (${GREEN}RViz configuration${WHITE}) listener with IP 10.0.0.$oa_ip...${WHITE}!"
echo -e "${WHITE}...${WHITE}"

echo -e "${GREEN}FINISHED WITH MAINTENANCE!${WHITE}"
