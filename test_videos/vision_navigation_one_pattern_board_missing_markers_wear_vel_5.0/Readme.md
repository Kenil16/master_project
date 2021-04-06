# Results from Gazebo from simulations
## _Vision based navigation_

This test was conducted to evaluate the performance of the vision based navigation of the drone using ArUco markers. Here the pose estimation will compared to the ground truth of the drone to see how well the pose esimation performs when the drone is moving around in the simulated environment. 

In this case, an ArUco marker board with missing markers have been used to stress test the system. In this case sensor fusion, which is based on pose esimates from the ArUco pose estimation, IMU and Barometer data, will be put to test in the simulated environment using the bottom camera of the drone. The forward velocity is set to 5 m/s.  

<p align="center">
  <img src="vision_navigation_one_pattern_board_missing_markers_wear_vel_5.0.gif" 
  width="80%">
</p>