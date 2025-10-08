#!/bin/bash

echo `export WITH_SIM=1 >> ~/.bashrc`
echo `export GAZEBO_MODEL_PATH=$HOME/ros_ws/install/yasarobo2925_26/share/yasarobo2925_26/models/:${GAZEBO_MODEL_PATH}' >> ~/.bashrc`