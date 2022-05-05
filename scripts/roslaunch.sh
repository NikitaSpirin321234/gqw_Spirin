#!/bin/bash

source ../catkin_ws/devel/setup.bash
package_name="mobot_gazebo"
launch_name="mobot.launch"

roslaunch $package_name $launch_name

