#!/bin/bash

#Save the path to ROS workspace 
ros_workspace="$PWD"

#Install dependencies for PX4
sudo apt install astyle build-essential ccache clang clang-tidy cmake cppcheck doxygen file g++ gcc gdb git lcov make ninja-build python3 python3-dev python3-pip python3-setuptools python3-wheel rsync shellcheck unzip xsltproc zip libeigen3-dev libopencv-dev libroscpp-dev protobuf-compiler python-pip python3-pip ninja-build gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev xvfb -y
sudo apt install python-catkin-tools python-rosdep2 -y

pip install --user argparse cerberus empy jinja2 numpy packaging pandas psutil pygments pyros-genmsg pyserial pyulog pyyaml setuptools six toml wheel rosdep
pip3 install --user --upgrade empy jinja2 numpy packaging pyros-genmsg toml pyyaml pymavlink

#Install ROS and mavros 
sudo rosdep init
sudo rosdep update
sudo apt install ros-melodic-desktop-full -y
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras -y

#Install GeographicLib datasets from your home folder
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod 755 install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

#Initialize PX4 SITL environment from your home folder
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
