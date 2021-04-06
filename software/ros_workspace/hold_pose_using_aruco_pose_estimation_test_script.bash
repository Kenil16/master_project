#!/bin/bash

#Specify data analysis directory, name of data file, name for the test and number of runs 
DIRECTORY="../../data/analyse_hold_pose_using_aruco_pose_estimation/test5_landingBoard3_noWind/"
data_file="../../data/hold_pose_using_aruco_pose_estimation.txt"
test_name="test"
iterations=10

#Run for nunmber of test times 
if [ ! -d "$DIRECTORY" ]; then
	echo "Dir does not exist"
	mkdir -p $DIRECTORY
else
	echo "Dir does exist"
fi

for (( c=1; c<=$iterations; c++ ))
do
	roslaunch px4 gazebo_sim_v1.0.launch worlds:=optitrack_big_board_onepattern.world drone_control_args:="hold_aruco_pose_test" x:=3.0 y:=-3.3 headless:=true gui:=false
	cat $data_file > "$DIRECTORY$test_name$c.txt"
	echo "Test $c completed!"
done

