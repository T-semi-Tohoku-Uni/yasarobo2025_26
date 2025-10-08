#!/bin/bash

cd /ros_ws/src
git clone https://github.com/keigo1216/ldrobot-lidar-ros2
cd ldrobot-lidar-ros2
git checkout humble
cd script
./create_udev_rules.sh