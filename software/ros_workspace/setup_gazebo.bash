#!/bin/bash

#source /opt/ros/melodic/setup.bash
source devel/setup.bash

source ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware/Tools/sitl_gazebo

#Set current location to be at Odense airport
export PX4_HOME_LAT=55.4719762
export PX4_HOME_LON=10.3248095
export PX4_HOME_ALT=7.4000000

