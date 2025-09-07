# WAI World - An open-source workspace for research and education.
WAI World repository with a selection of essential ROS packages.

![Alt text](wai_world.png?raw=true "WAI World")

**WAI World** (wai_world) is an open-source workspace, providing essential entities (representatives) for research and education. **Open Auditorium (OA)** (wai_oa) is a sub workspace of wai_world, focusing on the implementation of a human-centric eXtended Reality (XR) Blended Learning (BL) tool. In the presence of AI systems (AIS) and their rapidly evolving capabilities, core contributions of OA are its conceptual ethical conditions of use (COU) to responsibly use AI technology and, most of all, preserve natural human presence in education. Besides, OA aims for a large set of input modalities to enrich the interactive process of research, teaching and learning, ultimately leading to a more lively and expressive experience. The wai_world architecture, combined with the modularity of ROS, allows for a simple extension with additional representatives, encouraging further development by a large community. Designed for researchers, educators, and learners, particularly OA may serve as a playground for the implementation of more (existing or novel) AIS-assisted teaching and learning methods. Currently wai_world is implemented in ROS1 (Noetic), supported on Ubuntu 20.04.6 LTS. However, future plans involve an upgrade to ROS2, including a new world architecture.

**Please keep in mind** that this repository contains open source and untested software, provided "as is". It represents a prototype (originally designed and implemented in 2019) and does not reflect the latest standing of the workspace!

---

## 1. Installation of WAI World

### 1.1 Install And Prepare Ubuntu And ROS
To use the world architecture, install Ubuntu 20.04.6LTS and ROS1 Noetic.
For the installation of ROS1 Noetic, refer to the [**installation guide**](https://wiki.ros.org/noetic/Installation/Ubuntu).

To avoid any conflicts with regional settings (language and format), change the language format in Ubuntu's "Settings"
under "Region & Language" and "Formats" to "United States".

First, make sure your Debian package index is up-to-date:
```
sudo apt-get update
```
Purge any unncessary packages:
```
sudo apt-get purge modemmanager
```

---

### 1.2 Prepare ROS
Make sure your Debian package index is up-to-date:
```
sudo apt-get update
```
Install additional required ROS dependencies: 
```
sudo apt-get install ros-noetic-camera-info-manager ros-noetic-cv-bridge ros-noetic-rqt-multiplot ros-noetic-rosbridge-* python3-tornado ros-noetic-ddynamic-reconfigure ros-noetic-image-transport ros-noetic-image-transport-plugins ros-noetic-octomap* ros-noetic-geographic-* ros-noetic-octomap-ros ros-noetic-zbar-ros ros-noetic-transmission-interface ros-noetic-joint-limits-interface ros-noetic-gazebo-ros ros-noetic-ros-control ros-noetic-gazebo-ros-control ros-noetic-control-toolbox ros-noetic-ros-controllers ros-noetic-position-controllers ros-noetic-velocity-controllers ros-noetic-effort-controllers ros-noetic-joint-state-controller ros-noetic-controller-* ros-noetic-moveit*
```
Install additional libraries, required for multimodal input:
```
sudo apt-get install libcurlpp-dev libhidapi-* libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libsndfile1-dev libx11-dev libspnav-dev libspdlog-dev libsdl2-* festival ffmpeg libv4l-dev v4l-utils
```
Install additional tools, required for development:
```
sudo apt-get install ssh* net-tools meld qtcreator qtmultimedia5-* xmlstarlet python3-pip
```

---

### 1.3 Installation
First, create a ROS workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin build
```
Second, clone this wai_world repository via https:
```
cd ~/catkin_ws/src
git clone https://github.com/weraleiso/wai_world.git
```
Third, build the workspace (if building the first time, add a job limit!):
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin build -j2
```

---

### 1.4 Setup Additional Libraries For AIS-Assistance
To enable natural language processing (NLP), and prompting, based on Large Language Models (LLM), the world architecture utilizes drivers from PICOVOICE.
Currently, the (voice) prompt interactions in OA are based on the llama-2-7b-chat-342.pllm chat model.
To use the interactions, please setup your own account and services at the [**PICOVOICE Developer Console**](https://console.picovoice.ai/signup).
- Once you are registered, provide your own access key in the *wai_world_gazebo_spawn.launch* file (line 56-68) in the **wai_world_gazebo** package.
- Additionally, download the *llama-2-7b-chat-342.pllm* file via your account, and move it to the *.../picovoice_ros/picovoice_driver/resources/models* folder in the **wai_world_tools** package.
- Finally, check for proper versions of the [**cheetah**](https://github.com/Picovoice/cheetah), [**orca**](https://github.com/Picovoice/orca), [**picollm**](https://github.com/Picovoice/picollm), [**porcupine**](https://github.com/Picovoice/porcupine), [**pvrecorder**](https://github.com/Picovoice/pvrecorder), [**pvspeaker**](https://github.com/Picovoice/pvspeaker), and [**rhino**](https://github.com/Picovoice/rhino) engines. At some point you may want to upgrade to more recent versions of the engine, however, keep in mind that you eventually have to adapt the interfaces in the **picovoice_ros** package.

---

### 1.5 Setup The HTC Vive VR HMD
Setting up VR hardware in Ubuntu running Steam can be tricky. You should always keep SteamVR up to date and check for recent versions of OpenVR accordingly. For detailed instructions and a bunch of tipps for troubleshooting please check the **howto_install_wai_world_on_ubuntu_20_04_6_lts.txt** file in the **docs** folder of the **wai_world_bringup** package.

---

## 2. WAI World Packages

At the moment, **WAI World** holds the following packages:

- [**weraleiso/wai_world/wai_world_bringup**](https://github.com/weraleiso/wai_world/tree/master/wai_world_bringup) Bringup package to get things going.
- [**weraleiso/wai_world/wai_world_gazebo**](https://github.com/weraleiso/wai_world/tree/master/wai_world_gazebo) Gazebo package to provide physics simulation.
- [**weraleiso/wai_world/wai_world_launcher**](https://github.com/weraleiso/wai_world/tree/master/wai_world_launcher) A GUI-based launcher to help with configuration and launch of world.
- [**weraleiso/wai_world/wai_world_tools**](https://github.com/weraleiso/wai_world/tree/master/wai_world_tools) A collection of *modified* ROS packages (genuine and custom packages), required to properly run the world architecture.
- [**weraleiso/wai_world/wai_oa**](https://github.com/weraleiso/wai_world/tree/master/wai_oa) Open Auditorium (OA) is a sub-workspace of the wai_world workspace, implementing a human-centric eXtended Reality (XR) Blended Learning (BL) tool.
- [**weraleiso/wai_world/wai_marvin**](https://github.com/weraleiso/wai_world/tree/master/wai_marvin) MARVIN is a robotic representative and acts as assistive AIS in OA.
- [**weraleiso/wai_world/wai_ingenuity**](https://github.com/weraleiso/wai_world/tree/master/wai_ingenuity) INGENUITY is a robotic representative and facilitates virtual experimentation.

---

## References:
- Isop, W.A., 2024. Acceptance Evaluation of a Digital Extended Reality Teaching and Learning Tool to Support Constructivist-Oriented Teaching Methods in Engineering Education. Didacticum, 7(1), pp.47-67.
- Isop, W.A., 2025. Open Auditorium - A Human-Centric Extended Reality Blended Learning Tool. Frontiers in Artificial Intelligence, ???, p.??? [Manuscript submitted for publication].
