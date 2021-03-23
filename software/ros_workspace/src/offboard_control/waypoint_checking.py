#!/usr/bin/env python2

import numpy as np
from tf.transformations import  euler_from_quaternion
from geometry_msgs.msg import PoseStamped

class waypoint_checking():

    def __init__(self):
        pass
    
    def waypoint_check(self, uav_local_pose, setpoint_poseStamped = None, setpoint_xyzYaw = None, threshold=0.25):

        if not setpoint_poseStamped == None:
            angle = euler_from_quaternion([setpoint_poseStamped.pose.orientation.x,
                                           setpoint_poseStamped.pose.orientation.y,
                                           setpoint_poseStamped.pose.orientation.z,
                                           setpoint_poseStamped.pose.orientation.w])

            setpoint_x = setpoint_poseStamped.pose.position.x
            setpoint_y = setpoint_poseStamped.pose.position.y
            setpoint_z = setpoint_poseStamped.pose.position.z
            setpoint_yaw = angle[2]
        else:
            setpoint_x = setpoint_xyzYaw[0]
            setpoint_y = setpoint_xyzYaw[1]
            setpoint_z = setpoint_xyzYaw[2]
            setpoint_yaw = np.deg2rad(setpoint_xyzYaw[3])

        delta_x = uav_local_pose.pose.position.x - setpoint_x
        delta_y = uav_local_pose.pose.position.y - setpoint_y
        delta_z = uav_local_pose.pose.position.z - setpoint_z

        ori =  euler_from_quaternion([uav_local_pose.pose.orientation.x,
                                      uav_local_pose.pose.orientation.y,
                                      uav_local_pose.pose.orientation.z,
                                      uav_local_pose.pose.orientation.w])

        delta_yaw = ori[2]-setpoint_yaw
        error = np.sqrt(np.power(delta_x,2) + np.power(delta_y,2) + np.power(delta_z,2) + np.power(delta_yaw,2))
        #print(error)

        if error < threshold:
            return True
        else:
            return False

if __name__ == "__main__":
    wc = waypoint_checking()
