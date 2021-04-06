# Results from Gazebo simulations
## _Hold pose using the landing station three ArUco marker board_

This test was conducted to investigate the abillity of the drone to hold its pose in front of the landing board three. The outcome of this would yield the performance of the control algorithms for pose control and conclude if further optimizations in pose control have to be performed.

The drone has to keep its pose approximatly 1 meter away from the ArUco marker board.  

The reason to have three different landing boards is to see how the ArUco pose estimation improves when the number of ArUco markers increases in the image. This is because more ArUco markers yields more 3D pose estimations and improves the performance of the algorithms for pose estimation.  

<p align="center">
  <img alt="Light" src="analyse_hold_pose_using_aruco_pose_estimation_landing_station3.gif" width="80%">
</p>
