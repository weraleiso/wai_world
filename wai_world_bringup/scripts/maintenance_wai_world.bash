#!/usr/bin/env bash

# (1) Login at remote host
# (2) Remove catkin_ws
# (3) Copy local catkin_ws into remote HOME folder
# (4) Re-Compile whole workspace with catkin build

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
WHITE='\033[0;37m'

echo -e "${CYAN}WAI WORLD - MAINTENANCE ${WHITE}[ias@10.0.0.${RED}(LO)${WHITE}]"
echo -e "${WHITE}The ${RED}whole catkin workspace ${WHITE}is going to be renewed! ${RED}Are you sure? Press any key to confirm!${WHITE}"
read

echo -e "${WHITE}Input Audience IP Last Octet (LO):"
read oa_ip_lo
oa_ip=$((oa_ip_lo+0))
echo -e "Selected Audience IP: 10.0.0.${RED}$oa_ip${WHITE}"

sshpass -v -p ias ssh ias@10.0.0.$oa_ip 'source /opt/ros/noetic/setup.bash; rm -rf /home/ias/catkin_ws; mkdir -p /home/ias/catkin_ws/src; cd /home/ias/catkin_ws; catkin init; catkin build'
sshpass -v -p ias scp -r /home/ias/catkin_ws/src ias@10.0.0.$oa_ip:/home/ias/catkin_ws
sshpass -v -p ias ssh ias@10.0.0.$oa_ip 'source /opt/ros/noetic/setup.bash; source /home/ias/catkin_ws/devel/setup.bash; cd /home/ias/catkin_ws; catkin build; source /home/ias/catkin_ws/devel/setup.bash'
echo -e "${GREEN}Re-sourced setup files in workspace. Finished with maintenance! Onwards and upwards!${WHITE}"
