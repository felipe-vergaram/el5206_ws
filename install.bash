#!/bin/bash

sudo apt update

echo "Add ROS repository"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


echo "Install curl"
sudo apt install -y curl

echo "Add keys to ROS repository"
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo "Update repository"
sudo apt update

echo "Install ROS Noetic"
sudo apt install -y ros-noetic-desktop-full

echo "Source installation file"
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

echo "Install dependencies for building ROS packages"
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

echo "Install and initialize rosdep"
sudo apt install -y python3-rosdep
rosdep init
rosdep update

echo "Install some useful packages used in EL5206"
sudo apt install -y ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-teleop ros-noetic-slam-gmapping ros-noetic-map-server

echo "Build workspace"
catkin_make
echo "source ~/el5206_ws/devel/setup.bash" >> ~/.bashrc
source devel/setup.sh

echo "Exporting some variables"
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

echo "Installation ready!"
