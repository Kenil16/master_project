# Vision based navigation and precision landing of a drone
## _An autonomous implementation based on ROS, PX4 and OpenCV_

This project proposes methods for navigation of an unmanned aerial vehicle (UAV) utilizing computer vision. This is
achieved using ArUco marker boards which are used for pose estimation of the UAV. The goal is to have a UAV to fly
autonomously using GPS in outdoor environments from where missions can be executed. When the UAV needs to recharge,
a reliable GPS to vision transition is performed where the UAV uses its front camera to detect an ArUco marker board
located on a wall to the entrance of a building. When the UAV is of a certain distance in front of this board located on the
wall, it will start using its bottom camera for navigation in indoor GPS denied environments. In the indoor environment, a
precise vision based landing can then be performed when it needs to recharge, by using its front camera for pose estimation
of ArUco marker boards located in front of the landing stations. For vision based navigation, sensor fusion will be used
for pose optimizations. An illustration of this can be seen in the video below. The master thesis for this project can be accessed using [report](https://github.com/Kenil16/master_project/blob/master/master_report/Head/report.pdf).

<p align="center">
  <img alt="Light" src="https://github.com/Kenil16/master_project/blob/master/test_videos/raw_videos/final_test.gif" 
  width="80%">
</p>

To achieve autonomous flight for offboard control, the robot operating system (ROS) is used along with the PX4 autopilot.
Using ROS, a number of nodes for drone control, autonomous flight, ArUco marker detection and sensor fusion can be
run concurrently. The ArUco marker detection and pose estimation are accomplished by using OpenCV and an unscented
Kalman filter for sensor fusion. The implementation has been tested in simulation using Gazebo and real flight using a
Holybro Pixhawk 4 mini QAV250 Quadcopter with a Raspberry Pi 4B which was used as companion computer where an
OptiTrack system has been used for ground truth tracking of the UAV.

Results show that the implementation works well, where the UAV was able to make GPS to vision transitions even in
strong wind as well as its ability to use a minimum amount of ArUco markers located on the ground for vision based
navigation. The latter was accomplished by successfully implementation of sensor fusion where IMU, barometer and
vision data were fused together to achieve reliable pose estimates. The mean error of the precision landings were found
to be in the order of centimeters which concludes a reliable implementation of the vision based landings.

# Table of content
- [Installation](#Installation)
- [How to use](#How-to-use)
- [Simulations](#Simulations)

# Installation
This project has been implemented on an Ubuntu 18.04 machine using the  ROS, Gazebo and OpenCV. It relies on the following software which must be installed to make it work properly.  

You can either download the software or clone it directly from your terminal. To use the latter you must install [Git](https://git-scm.com) using the command line:
```bash
#Install Git
sudo apt install git

#Clone software
git clone https://github.com/Kenil16/master_project.git
```

Now the project can be installed with all dependencies by running the script [init_setup.bash](https://github.com/Kenil16/master_project/blob/master/software/ros_workspace/init_setup.bash). Now change directory and run the initialization script: 
```bash
#Change directory and execute the initialization script
cd software/ros_workspace/
. ./init_setup.bash
```

Now the installation and setup of the system are completed. If one wants to install the necessary software one-by-one, the following steps must be completed.

Install PX4 dependencies:
```bash
#Install a packages via apt 
sudo apt install astyle build-essential ccache clang clang-tidy cmake cppcheck doxygen file g++ gcc gdb git lcov make ninja-build python3 python3-dev python3-pip python3-setuptools python3-wheel rsync shellcheck unzip xsltproc zip libeigen3-dev libopencv-dev libroscpp-dev protobuf-compiler python-pip python3-pip ninja-build gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer10-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev xvfb python-rosdep2

#Install packages via pip
pip install --user argparse cerberus empy jinja2 numpy packaging pandas psutil pygments pyros-genmsg pyserial pyulog pyyaml setuptools six toml wheel rosdep

#Install packages via pip3
pip3 install --user --upgrade empy jinja2 numpy packaging pyros-genmsg toml pyyaml pymavlink
```

Install [texmaker](https://www.xm1math.net/texmaker/) and [miktex](https://miktex.org/howto/install-miktex-unx) for plots in latex format and report reading 
```bash
#Install texmaker
sudo apt install texmaker

#Register GPG key
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D6BC243565B2087BC3F897C9277A7293F59E4889

#Register installation source
echo "deb http://miktex.org/download/ubuntu bionic universe" | sudo tee /etc/apt/sources.list.d/miktex.list

#Install MiKTeX
sudo apt-get update
sudo apt-get install miktex
```

[ROS](http://wiki.ros.org/Installation/Ubuntu) melodic is used in this project and can be installed together with Gazebo:
```bash
#Init workspace
sudo rosdep init
sudo rosdep update

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

Install dependencies to run software scripts:
```bash
sudo apt update

sudo apt install ros-melodic-hector-gazebo-plugins dvipng

pip install pathlib numpy pandas opencv-contrib-python scipy pymavlink matplotlib
```

[Update Gazebo](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0) version (tested on 9.16.0)
```bash
#Setup your computer to accept software from packages.osrfoundation.org.
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

#Setup keys
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

#Update version
sudo apt-get update
sudo apt-get install gazebo9

#Otherwise gazebo will not run
sudo apt upgrade libignition-math2
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

#Make symlink to user software models to PX4 SITL
cd software/ros_workspace/
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
To setup the system you have to run the following configuration file from your terminal to link PX4 to the ROS environment:
```bash
#Be in your ROS workspace and run the setup script
cd software/ros_workspace/
. ./setup_gazebo.bash
```

To run the program you have to execute the following command from your terminal:
```bash
#Run roslaunch which starts all the nodes in the system 
roslaunch px4 gazebo_sim_v1.0.launch worlds:=optitrack_big_board_onepattern_missing_markers.world drone_control_args:="idle" x:=-21.0 y:=0
```
Here you can define the world from a number of possible [setups](https://github.com/Kenil16/master_project/tree/master/software/ros_workspace/PX4-software/worlds). The UAV will be in idle state by default, but that can be changed to some mission if prefered. This will be discussed in the [report](https://github.com/Kenil16/master_project/blob/master/master_report/Head/report.pdf).

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
