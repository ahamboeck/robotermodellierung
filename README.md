# ROS Pharmaceutical Robotics Suite

## Overview
This repository contains a comprehensive suite of ROS Noetic packages for controlling robotic operations in pharmaceutical environments. The system includes three different robots, all equipped with motion planning capabilities through MoveIt and interactive control via a Human-Machine Interface (HMI).

## Prerequisites
Before you can use these ROS packages, ensure you have the following installed on your Ubuntu 20.04 LTS system:
- **ROS Noetic**
  - Install ROS Noetic by following the official guide available at [ROS Installation](http://wiki.ros.org/noetic/Installation/Ubuntu).

## Dependencies
This package requires the following software dependencies:
- **GLFW3** for graphical interface support in the HMI:
```bash
sudo apt-get install libglfw3-dev
```
- **MoveIt** for robotic motion planning:
```bash
sudo apt install ros-noetic-moveit
```

## Installation
Follow these detailed steps to set up and configure your ROS workspace for the pharmaceutical robotics suite:

1. **Create and initialize the ROS workspace**:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

2. **Clone this repository into your workspace**:
```bash
cd ~/catkin_ws/src
git clone https://github.com/ahamboeck/robotermodelierung.git
```

3. **Install all necessary ROS dependencies**:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

4. **Build the ROS packages**:
```bash
cd ~/catkin_ws
catkin_make
```

5. **Source the environment to ensure all packages are properly integrated**:
```bash
source devel/setup.bash
```

## Usage
To operate the robots using the configurations provided in this suite, you can start the demo environment which includes all three robots using the following command:
```bash
roslaunch triple_robot_config demo.launch
```