#!/bin/bash

# source ../catkin_ws/devel/setup.bash
source /opt/ros/noetic/setup.bash
export ROS_HOSTNAME=`ip route | grep -Eom 1 "([0-9]{1,3}[\.]){3}[0-9]{1,3}" -C 1`
export ROS_MASTER_URI=http://$1.local:11311
export ROS_MASTER=$1
export VEHICLE_NAME=$1
topic="/$1/camera_node/image/compressed"

# . ../app/remote.sh autobot05

cd ../catkin_ws/src/mobot/mobot_gazebo/bagfiles

# trap "rosnode kill /mybag" SIGINT

rosbag record $topic --chunksize=256 #__name:=my_bag
