# Vision based navigation and precision landing of a drone
## _An autonomous implementation based on ROS, PX4 and OpenCV_

<!--
<p align="center">
  <img alt="Light" src="master_report/Figures/px4_autopilot_logo.png" width="20%">
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 
  <img alt="Dark" src="master_report/Figures/opencv_logo.png" width="10%">
  &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 
  <img alt="Dark" src="master_report/Figures/ros_gazebo_logo.png" width="30%">
</p>
-->

#
TODO (Project overview)

# Table of content
- [Installation](#Installation)
- [How to use](#How-to-use)
- [Simulations](#Simulations)
- [License](#License)


# Installation
This project has been implemented on an Ubuntu 18.04 machine using robot operating system (ROS), Gazebo and OpenCV. It relies on the following sofware which must be installed to make it work proberly.  

You can either download the software or clone it directly from your terminal. To use the latter you must install [Git](https://git-scm.com) using the command line:
```bash
#Install Git
sudo apt install git

#Clone software
git clone https://github.com/Kenil16/master_project.git
```

[VIM](https://www.vim.org/) is highly recommended as text editor:
```bash
#Install vim
sudo apt install vim
```

Now the project can be installed with all dependencies by running the scipt [init_setup.bash](https://github.com/Kenil16/master_project/blob/master/software/ros_workspace/init_setup.bash). Now change directory and run the initialization script: 
```bash
#Change diretory and execute initialization script
cd software/ros_workspace/
. ./init_setup.bash
```

Now the installation and setup of the system are completed. If one wants to install the necessary software one-by-one, the following stepts must be completed.

Install PX4 dependencies:
```bash
#Install a packages via apt 
sudo apt install astyle build-essential ccache clang clang-tidy cmake cppcheck doxygen file g++ gcc gdb git lcov make ninja-build python3 python3-dev python3-pip python3-setuptools python3-wheel rsync shellcheck unzip xsltproc zip libeigen3-dev libopencv-dev libroscpp-dev protobuf-compiler python-pip python3-pip ninja-build gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer10-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev xvfb python-rosdep2

#Install packages via pip
pip install --user argparse cerberus empy jinja2 numpy packaging pandas psutil pygments pyros-genmsg pyserial pyulog pyyaml setuptools six toml wheel rosdep

#Install packages via pip3
pip3 install --user --upgrade empy jinja2 numpy packaging pyros-genmsg toml pyyaml pymavlink
```

Install dependencies to run sofware scripts:
```
sudo apt update

apt install python-cv-bridge ros-melodic-hector-gazebo-plugins

pip install pathlib numpy pandas opencv-contrib-python scipy pymavlink
```

[ROS](http://wiki.ros.org/Installation/Ubuntu) melodic is used in this project and can be installed together with Gazebo:
```bash
#Setup your sources.list
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#Set up your keys
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#Installation
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full
```

[Mavros](http://wiki.ros.org/mavros) can be installed using:
```bash
#Install mavros along with extra packages
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras -y
```

Install GeographicLib datasets:
```bash
#Fetch the sofware, change permissions and execute script 
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod 755 install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

[PX4](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html) must be installed and this setup assumes PX4 SITL to be installed from the /home directory. Another installation diretcory can easily be made, but then the symbolics links must be changed accordingly:
```bash
#Initialize PX4 SITL environment from your home folder
cd ~
mkdir PX4_SITL

cd PX4_SITL
git clone https://github.com/PX4/Firmware.git --branch v1.11.0

cd Firmware
git submodule update --init --recursive

#Make symlink to user software for link models to PX4 SITL
cd $ros_workspace
ln -s $PWD/PX4-software/init.d-posix/* /home/$USER/PX4_SITL/Firmware/ROMFS/px4fmu_common/init.d-posix/
ln -s $PWD/PX4-software/mixers/* /home/$USER/PX4_SITL/Firmware/ROMFS/px4fmu_common/mixers/
ln -s $PWD/PX4-software/models/* /home/$USER/PX4_SITL/Firmware/Tools/sitl_gazebo/models/
ln -s $PWD/PX4-software/worlds/* /home/$USER/PX4_SITL/Firmware/Tools/sitl_gazebo/worlds/
ln -s $PWD/PX4-software/launch/* /home/$USER/PX4_SITL/Firmware/launch/

#Build the PX4 SITL firmware
cd /home/$USER/PX4_SITL/Firmware
DONT_RUN=1 make px4_sitl_default gazebo
```

# How to use
The first time your are using the sofware, the ROS workspace has to be initialzed:
```bash



```

# Simulations 

## Simulations from tests 
  - [GPS2Vision pose estimation](https://github.com/Kenil16/master_project/tree/master/test_videos/analyse_GPS2Vision_aruco_pose_estimation)

  - [Hold pose using ArUco pose estimation for GPS2Vision board witn no wind](https://github.com/Kenil16/master_project/tree/master/test_videos/analyse_hold_pose_using_aruco_pose_estimation_gps2vision_noWind)

  - [Hold pose using ArUco pose estimation for GPS2Vision board witn 5-7 m/s wind](https://github.com/Kenil16/master_project/tree/master/test_videos/analyse_hold_pose_using_aruco_pose_estimation_gps2vision_5-7ms_wind)

  - [Hold pose using ArUco pose estimation for landing station one](https://github.com/Kenil16/master_project/tree/master/test_videos/analyse_hold_pose_using_aruco_pose_estimation_landing_station1)

  - [Hold pose using ArUco pose estimation for landing station two](https://github.com/Kenil16/master_project/tree/master/test_videos/analyse_hold_pose_using_aruco_pose_estimation_landing_station2)

  - [Hold pose using ArUco pose estimation for landing station three](https://github.com/Kenil16/master_project/tree/master/test_videos/analyse_hold_pose_using_aruco_pose_estimation_landing_station3)

  - [GPS2Vision transition with no wind](https://github.com/Kenil16/master_project/tree/master/test_videos/analyse_gps2vision_noWind)

  - [GPS2Vision transition with 5-7 m/s wind](https://github.com/Kenil16/master_project/tree/master/test_videos/analyse_gps2vision_5-7ms_wind)

  - [GPS2Vision transition with 7-10 m/s wind](https://github.com/Kenil16/master_project/tree/master/test_videos/analyse_gps2vision_7-10ms_wind)

  - [Vision navigation using full ArUco board with 1.0 m/s horizontal velocity](https://github.com/Kenil16/master_project/tree/master/test_videos/vision_navigation_full_marker_board_vel_1.0)

  - [Vision navigation using ArUco board with missing markers with 5.0 m/s horizontal velocity](https://github.com/Kenil16/master_project/tree/master/test_videos/vision_navigation_one_pattern_board_missing_markers_wear_vel_5.0)

  - [Vision landing with 0.5 m/s vertical velocity](https://github.com/Kenil16/master_project/tree/master/test_videos/vision_landing_precision_and_accuracy_vertical_vel_0.5_max_error_0.05)


# License
TODO (Lincense)