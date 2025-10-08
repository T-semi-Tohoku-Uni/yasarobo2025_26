#!/bin/bash

cd /ros_ws/src
git clone https://github.com/keigo1216/inrof2025_ros_type.git

cd /ros_ws/src
git clone https://github.com/keigo1216/ldrobot-lidar-ros2
cd ldrobot-lidar-ros2
git checkout humble
cd script
./create_udev_rules.sh

echo `export WITH_SIM=1 >> ~/.bashrc`
echo `export GAZEBO_MODEL_PATH=$HOME/ros_ws/install/yasarobo2925_26/share/yasarobo2925_26/models/:${GAZEBO_MODEL_PATH}' >> ~/.bashrc`