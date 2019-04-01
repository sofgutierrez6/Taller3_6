#!/bin/bash

if [ $_ = $0 ]
then
  echo 'This script should not be executed as program'
  echo 'Please, source it'
  echo
  echo 'Example:'
  echo "$ source ${BASH_SOURCE[0]}"
  echo
  exit 1
fi

echo 'Updating OS'
sudo apt-get update
if [ $? -eq 0 ]; then
  echo 'OK!'
else
  echo 'Error Updating OS'
  return 
fi

echo

echo 'Upgrading OS'
sudo apt-get update
if [ $? -eq 0 ]; then
  echo 'OK!'
else
  echo 'Error Upgrading OS'
fi

echo

echo 'Setting sources.list'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
if [ $? -eq 0 ]; then
  echo 'OK!'
else
  echo 'Error Setting sources.list'
fi

echo

echo 'Setting up your keys'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
if [ $? -eq 0 ]; then
  echo 'OK!'
else
  echo 'Error Setting up your keys'
fi

echo

echo 'Updating OS'
sudo apt-get update
if [ $? -eq 0 ]; then
  echo 'OK!'
else
  echo 'Error Updating OS'
  return 
fi

echo

echo 'Installing ROS'
sudo apt-get install ros-kinetic-desktop
if [ $? -eq 0 ]; then
  echo 'OK!'
else
  echo 'Error Installing ROS'
  return 
fi

echo

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo 'Installing Dependencies'
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
if [ $? -eq 0 ]; then
  echo 'OK!'
else
  echo 'Installing Dependencies'
  return 
fi

echo 'Setting the Catkin Workspace'

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

source devel/setup.bash
