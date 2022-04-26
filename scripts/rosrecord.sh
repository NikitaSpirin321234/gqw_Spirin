#!/bin/bash

source ../catkin_ws/devel/setup.bash
topic="/spcbot/camera/image_raw"

cd ../catkin_ws/src/mobot/mobot_gazebo/bagfiles

rosbag record $topic /topic __name:=my_bag
