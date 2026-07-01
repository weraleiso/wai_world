#!/usr/bin/env bash

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
WHITE='\033[0;37m'



###############################
# 1. Installation of WAI World
###############################


# 1.1 Prepare Ubuntu (Manual Installation)
sudo apt-get update
sudo apt-get purge modemmanager --yes
sudo apt-get purge thunderbird* --yes
sudo apt-get purge firefox* --yes


# 1.2 Prepare And Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl --yes # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full --yes
echo "source /opt/ros/noetic/setup.bash" >> /home/ias/.bashrc
source /home/ias/.bashrc

sudo apt-get install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools --yes
sudo rosdep init
rosdep update

# Install additional ROS packages:
sudo apt-get install ros-noetic-camera-info-manager ros-noetic-cv-bridge ros-noetic-rqt-multiplot ros-noetic-rosbridge-* python3-tornado ros-noetic-ddynamic-reconfigure ros-noetic-image-transport ros-noetic-image-transport-plugins ros-noetic-octomap* ros-noetic-geographic-* ros-noetic-octomap-ros ros-noetic-zbar-ros ros-noetic-transmission-interface ros-noetic-joint-limits-interface ros-noetic-gazebo-ros ros-noetic-ros-control ros-noetic-gazebo-ros-control ros-noetic-control-toolbox ros-noetic-ros-controllers ros-noetic-position-controllers ros-noetic-velocity-controllers ros-noetic-effort-controllers ros-noetic-joint-state-controller ros-noetic-controller-* ros-noetic-moveit* --yes

# Install additional libraries:
sudo apt-get install libcurlpp-dev libhidapi-* libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libsndfile1-dev libx11-dev libspnav-dev libspdlog-dev libsdl2-* festival ffmpeg libv4l-dev v4l-utils --yes

# Install developer tools:
sudo apt-get install gparted ssh* pssh net-tools meld qtcreator qtmultimedia5-* xmlstarlet cutycapt python3-pip --yes

# Re-Install LibreOffice (to prevent package conflicts with ooo-dev-tools)
sudo apt-get purge libreoffice* --yes
sudo apt-get autoremove --yes
sudo apt-get install libreoffice --yes
sudo apt-get install default-jre libreoffice-java-common --yes

# Install LibreOffice ODP Dev Tools
pip install ooo-dev-tools


# 1.3 Prepare Workspace And Install WAI World Package
source /opt/ros/noetic/setup.bash
mkdir -p /home/ias/catkin_ws/src
cd /home/ias/catkin_ws
catkin init
catkin build
source /home/ias/catkin_ws/devel/setup.bash
cd /home/ias/catkin_ws/src
git clone https://github.com/weraleiso/wai_world.git
cd /home/ias/catkin_ws
catkin build -j2



# (Optional) Customize Ubuntu Appearance, Sound, and Power Management

# Set desktop background color to black
gsettings set org.gnome.desktop.background picture-uri none
gsettings set org.gnome.desktop.background primary-color '#000000'

# Show battery percentage
gsettings set org.gnome.desktop.interface show-battery-percentage true

# Disable sleep when inactive, disable sleep and hybernation
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-timeout 0
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing'
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout 0
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

# Update PulseAudio and UPower Settings
sudo cp /home/ias/catkin_ws/src/wai_world/wai_world_bringup/scripts/default.pa /etc/pulse/default.pa
sudo cp /home/ias/catkin_ws/src/wai_world/wai_world_bringup/scripts/UPower.conf /etc/UPower/UPower.conf

