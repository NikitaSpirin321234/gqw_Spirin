#!/bin/bash

source ../catkin_ws/devel/setup.bash
topic="/spcbot/camera/image_raw"

rosrun image_view image_view image:=$topic

echo roslaunch

