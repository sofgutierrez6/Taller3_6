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

DEF_PATH="$HOME/catkin_ws"
ORIG_PATH=$( pwd )

echo
echo 'Starting ros-pacman server...'
echo

echo 'Type the Pacman workspace path:'
read -p "[default (Enter): ${DEF_PATH}]: " WS_PATH
if [ -z "$WS_PATH" ]
then
  WS_PATH=$DEF_PATH
fi

echo

if [ -d "$WS_PATH" ]; then
  echo "Folder '$WS_PATH' found"
else
  echo "Folder '$WS_PATH' not found"
  echo "Aborting"
  return
fi

echo

export ROS_IP=$( hostname -I )
export ROS_IP=${ROS_IP//[[:blank:]]/}

echo 'Server ready.'
echo "Use the IP address ${ROS_IP} in your Pacman Controller"
echo

echo "Press g for 'game mode' and c for 'challenge mode'"
read MODE

if [ $MODE != "c" -a $MODE != "g" ]
then echo "Wrong mode. Select g for 'game mode' and c for 'challenge mode'"
return
fi

echo
if [ $MODE == "c" ]
then echo "Enter map name" 
read MAP 
fi

echo "Do you want to run with --m (mute) option? (y or n)"
read ARGS
echo

if [ $ARGS == y ]
then MUTE="m"
else MUTE=""
fi

if pgrep -x "roscore" > /dev/null
then
  echo "'roscore' process found"
else echo "Running 'roscore'..."
gnome-terminal -e 'roscore'
fi

export ROS_MASTER_URI="http://${ROS_IP}:11311"
export ROS_MASTER_URI=${ROS_MASTER_URI//[[:blank:]]/}

cd $WS_PATH
source devel/setup.bash
rosrun pacman pacman_world --$MODE $MAP --$MUTE

cd $ORIG_PATH
