#!/bin/bash

cd ../..;
source devel/setup.bash
catkin_make

source devel/setup.bash

roslaunch visus_tcp_bridge bridge.launch