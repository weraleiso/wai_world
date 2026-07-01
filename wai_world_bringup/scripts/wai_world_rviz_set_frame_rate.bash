#!/bin/bash
var=$1
awk -v awk_var="$var" '{gsub("Frame Rate: 20",awk_var)}1' ~/catkin_ws/src/wai_world/wai_world_bringup/rviz/wai_world_template.rviz > ~/catkin_ws/src/wai_world/wai_world_bringup/rviz/wai_world.rviz
