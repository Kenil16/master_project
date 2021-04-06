#!/bin/bash

#Specify data analysis directory, name of data file, name for the test and number of runs 
DIRECTORY="../../data/analyse_GPS2Vision/test5_7-10ms_wind/"
data_file="../../data/gps2vision.txt"
test_name="test"
iterations=30

#Run for nunmber of test times 
if [ ! -d "$DIRECTORY" ]; then
	echo "Dir does not exist"
	mkdir -p $DIRECTORY
else
	echo "Dir does exist"
fi

for (( c=1; c<=$iterations; c++ ))
do
	roslaunch px4 gazebo_sim_v1.0.launch worlds:=optitrack_big_board_onepattern.world drone_control_args:="GPS2Vision_test" x:=-21.0 y:=0.0 headless:=true gui:=false
	cat $data_file > "$DIRECTORY$test_name$c.txt"
	echo "Test $c completed!"
done

