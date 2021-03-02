#!/bin/bash

#source /opt/ros/melodic/setup.bash
source devel/setup.bash

source ~/PX4_SITL/Firmware/Tools/setup_gazebo.bash ~/PX4_SITL/Firmware ~/PX4_SITL/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_SITL/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_SITL/Firmware/Tools/sitl_gazebo
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/devel/lib/

#Set current location to be at Odense airport
export PX4_HOME_LAT=55.4719762
export PX4_HOME_LON=10.3248095
export PX4_HOME_ALT=7.4000000

