#!/bin/bash

#Save the path to ROS workspace 
ros_workspace="$PWD"

#Initialize PX4 SITL environment from home folder
cd ~
mkdir PX4_SITL
cd PX4_SITL
git clone https://github.com/PX4/Firmware.git --branch v1.11.0
cd Firmware
git submodule update --init --recursive

#Make symlink to user software
cd $ros_workspace
ln -s $PWD/PX4-software/init.d-posix/* /home/$USER/PX4_SITL/Firmware/ROMFS/px4fmu_common/init.d-posix/
ln -s $PWD/PX4-software/mixers/* /home/$USER/PX4_SITL/Firmware/ROMFS/px4fmu_common/mixers/
ln -s $PWD/PX4-software/models/* /home/$USER/PX4_SITL/Firmware/Tools/sitl_gazebo/models/
ln -s $PWD/PX4-software/worlds/* /home/$USER/PX4_SITL/Firmware/Tools/sitl_gazebo/worlds/
ln -s $PWD/PX4-software/launch/* /home/$USER/PX4_SITL/Firmware/launch/

#Place 3d models in the PX4 firmware (-R recursively and -T not source directory)
#cd $ros_workspace
#cp -RT PX4-software/models/ /home/$USER/PX4_SITL/Firmware/Tools/sitl_gazebo/models
#cp -RT PX4-software/worlds/ /home/$USER/PX4_SITL/Firmware/Tools/sitl_gazebo/worlds
#cp -RT PX4-software/init.d-posix /home/$USER/PX4_SITL/Firmware/ROMFS/px4fmu_common/init.d-posix/
#cp -RT PX4-software/mixers /home/$USER/PX4_SITL/Firmware/ROMFS/px4fmu_common/mixers

#Build the PX4 SITL firmware
cd /home/$USER/PX4_SITL/Firmware
DONT_RUN=1 make px4_sitl_default gazebo

cd $ros_workspace
