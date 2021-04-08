#!/bin/bash

#Save the path to ROS workspace 
ros_workspace="$PWD"

#Install dependencies for PX4
sudo apt install astyle build-essential ccache clang clang-tidy cmake cppcheck doxygen file g++ gcc gdb git lcov make ninja-build python3 python3-dev python3-pip python3-setuptools python3-wheel rsync shellcheck unzip xsltproc zip libeigen3-dev libopencv-dev libroscpp-dev protobuf-compiler python-pip python3-pip ninja-build gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev xvfb -y
sudo apt install python-catkin-tools python-rosdep2 -y

pip install --user argparse cerberus empy jinja2 numpy packaging pandas psutil pygments pyros-genmsg pyserial pyulog pyyaml setuptools six toml wheel rosdep
pip3 install --user --upgrade empy jinja2 numpy packaging pyros-genmsg toml pyyaml pymavlink

#Install texmaker
sudo apt install texmaker

#Register GPG key
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D6BC243565B2087BC3F897C9277A7293F59E4889

#Register installation source
echo "deb http://miktex.org/download/ubuntu bionic universe" | sudo tee /etc/apt/sources.list.d/miktex.list

#Install MiKTeX
sudo apt-get update
sudo apt-get install miktex

#Install ROS and mavros 
rosdep init
sudo rosdep fix-permissions
rosdep update

#Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#Set up your keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#Installation
sudo apt update
sudo apt install ros-melodic-desktop-full -y
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras -y

#Install dependencies to run software scripts
sudo apt update
sudo apt install ros-melodic-hector-gazebo-plugins dvipng
pip install pathlib numpy pandas opencv-contrib-python scipy pymavlink matplotlib

#Update Gazebo version
#Setup your computer to accept software from packages.osrfoundation.org.
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

#Setup keys
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

#Get newest version (testet with 9.16.0)
sudo apt-get update
sudo apt-get install gazebo9

#Otherwise gazebo will not run
sudo apt upgrade libignition-math2

#Install GeographicLib datasets from your home folder
cd ~
mkdir geographicLib_datasets
cd geographicLib_datasets
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod 755 install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

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

#Build the PX4 SITL firmware
cd /home/$USER/PX4_SITL/Firmware
DONT_RUN=1 make px4_sitl_default gazebo

#Now inizialize ros workspace
cd $ros_workspace
source /opt/ros/melodic/setup.bash
catkin_make
