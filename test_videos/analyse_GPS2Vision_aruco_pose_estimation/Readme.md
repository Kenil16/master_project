# Results from simulations
## _GPS2Vision ArUco pose estimation_

This test was conducted to estimate the ArUco pose based on the GPS2Vision board located on the wall. This was done to find the error in the estimated pose based on the ground truth of the drone in the simulated environment. Hence, an indication of the maximum allowed distance away from the board before the pose estimation becomes to bad, could be estimated. An idea of the maximum distance is critical when a transition between using GPS to vision based navigation is to be performed.

## Description of test
The drone was set to estimate the pose of the ArUco board for a giving set of locations defined in a grid struture of waypoints. For each waypiont, the drone is set to face the board and find the mean error of the estimated pose for a user defined number of seconds. This will be used to apply linear interpolation between the error on the giving waypoints to visualize how the error propagates as the drone moves futher away from the board. 

<p align="center">
  <img alt="Light" src="analyse_GPS2Vision_aruco_pose_estimation.gif" width="80%">
</p>

