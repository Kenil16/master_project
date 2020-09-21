#!/bin/bash

#Save the path to ROS workspace 
ros_workspace="$PWD"

#Initialize PX4 SITL environment from home folder
cd ~
mkdir PX4_SITL
cd PX4_SITL
git clone https://github.com/PX4/Firmware.git
cd Firmware
git submodule update --init --recursive

#Place 3d models in the PX4 firmware (-R recursively and -T not source directory)
cd $ros_workspace
cp -RT 3D-models/models/ /home/$USER/PX4_SITL/Firmware/Tools/sitl_gazebo/models
cp -RT 3D-models/worlds/ /home/$USER/PX4_SITL/Firmware/Tools/sitl_gazebo/worlds

