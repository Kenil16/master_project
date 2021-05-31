# Results from Gazebo from simulations
## _GPS2Vision transition with no wind_

This test was conducted to see how the drone performs when going from GPS to vision based navigation. In this state, the drone has to locate the ArUco marker board located on the wall after flying to a user defined waypoint using the GPS. Because the GPS comes with a certain error, the final waypoint has been set ramdomly with a variation of +-6 meters in the x and y positions to illustrate this uncertainty. This is to stress test the drone's abillity to find the board with this error in mind. When the drone finds the board, it is set to navigate to it still using GPS as navigation. It will continue doing so, until it is close enough to the board on the wall (GPS2Vision board) to make a reliable GPS2Vision transition. 

The drone will be placed twenty meters away in front of the GPS2Vision board where this simulation includes no wind.

<p align="center">
  <img src="analyse_gps2vision_noWind.gif" 
  width="80%">
</p>