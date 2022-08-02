#!/bin/bash

# source ../catkin_ws/devel/setup.bash

. ../app/remote.sh autobot03

rosnode kill /my_bag
# kill -9 `pgrep record`
