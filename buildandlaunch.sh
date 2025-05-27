#!/bin/bash

cd ../..;
source devel/setup.bash
catkin_make

source devel/setup.bash

roslaunch visus_ros_bridge bridge.launch