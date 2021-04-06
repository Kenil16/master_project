#!/bin/bash

#Specify data analysis directory, name of data file, name for the test and number of runs 
DIRECTORY="../../data/analyse_vision_navigation_pose_estimation/test5_one_pattern_missing_markers_wear_board/"
data_file="../../data/sensor_fusion_data.txt"
test_name="test"
iterations=20

#Run for nunmber of test times 
if [ ! -d "$DIRECTORY" ]; then
	echo "Dir does not exist"
	mkdir -p $DIRECTORY
else
	echo "Dir does exist"
fi

for (( c=1; c<=$iterations; c++ ))
do
	roslaunch px4 gazebo_sim_v1.0.launch worlds:=optitrack_big_board_onepattern_missing_markers_wear.world drone_control_args:="vision_navigation_test" x:=-3.0 y:=0.0 headless:=true gui:=false
	cat $data_file > "$DIRECTORY$test_name$c.txt"
	echo "Test $c completed!"
done

