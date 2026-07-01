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
echo -e "${CYAN}*** AUDIENCE Copy Workspace *** ${WHITE}"
echo -e "${WHITE}==============================${WHITE}"
echo -e "${WHITE}...${WHITE}"
echo -e "${WHITE}The ${YELLOW}whole audience${WHITE} is going to be ${YELLOW}provided with a new catkin_ws folder${WHITE}!"
echo -e "${WHITE}...${WHITE}"
echo -e "${WHITE}Press any key to confirm...!${WHITE}"
read

for oa_ip in $(seq 1 10);
do
    if ping -c 1 10.0.0.$oa_ip &> /dev/null
    then
        echo -e "${YELLOW}Starting${WHITE} to maintain (${YELLOW}remove old workspace${WHITE}) listener with IP 10.0.0.$oa_ip...${WHITE}!"
        sshpass -v -p ias ssh ias@10.0.0.$oa_ip 'rm -rf /home/ias/catkin_ws'
        sshpass -v -p ias ssh ias@10.0.0.$oa_ip 'sync; sync; sync;'
        echo -e "${GREEN}Finished${WHITE} to maintain (${GREEN}remove old workspace${WHITE}) listener with IP 10.0.0.$oa_ip...${WHITE}!"
    else
        echo -e "${WHITE}Listener with IP 10.0.0.$oa_ip is ${YELLOW}not reachable!${WHITE}!"
    fi
done

date
echo -e "${YELLOW}Starting${WHITE} to maintain (${YELLOW}copying workspace${WHITE}) listener with IP 10.0.0.$oa_ip...${WHITE}!"
parallel-scp -r -h oa_listener_hosts.txt -l ias -p 10 -v -A /home/ias/catkin_ws /home/ias/catkin_ws
sync; sync; sync
echo -e "${GREEN}Finished${WHITE} to maintain (${GREEN}copying workspace${WHITE}) listener with IP 10.0.0.$oa_ip...${WHITE}!"
echo -e "${WHITE}...${WHITE}"
date
echo -e "${GREEN}FINISHED WITH MAINTENANCE!${WHITE}"
