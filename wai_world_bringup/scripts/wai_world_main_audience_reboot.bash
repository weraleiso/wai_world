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
echo -e "${CYAN}*** AUDIENCE Reboot *** ${WHITE}"
echo -e "${WHITE}==============================${WHITE}"
echo -e "${WHITE}...${WHITE}"
echo -e "${WHITE}The ${YELLOW}whole audience${WHITE} is going to be ${YELLOW}rebooted${WHITE}!"
echo -e "${WHITE}...${WHITE}"
echo -e "${RED}Press any key to confirm...!${WHITE}"
read

for oa_ip in $(seq 1 10);
do
    if ping -c 1 10.0.0.$oa_ip &> /dev/null
    then
        echo -e "${YELLOW}Starting${WHITE} to maintain (${YELLOW}reboot${WHITE}) listener with IP 10.0.0.$oa_ip...${WHITE}!"
        sshpass -v -p ias ssh -t ias@10.0.0.$oa_ip 'echo ias | sudo -S reboot'
        echo -e "${GREEN}Finished${WHITE} to maintain (${GREEN}reboot${WHITE}) listener with IP 10.0.0.$oa_ip...${WHITE}!"
    else
        echo -e "${WHITE}Listener with IP 10.0.0.$oa_ip is ${YELLOW}not reachable!${WHITE}!"
    fi
done

echo -e "${WHITE}...${WHITE}"
echo -e "${GREEN}FINISHED WITH MAINTENANCE!${WHITE}"

