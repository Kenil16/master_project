#!/usr/bin/env python2

import numpy as np
from tf.transformations import  euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion

class waypoint_generator:

    def __init__(self):
        pass

    """Method used in GPS2vision"""
    def generate_locate_marker_board_waypoints(self, uav_pose, start_x, end_x, steps_x, start_y, end_y, steps_y, start_yaw, end_yaw, steps_yaw):

        #Directions for initializing new places to search the GPS2Vision marker in meters if not found from last waypoint
        waypoints = []

        #Make grid of waypoints for the drone to look for the marker board 
        x_ = [i for i in range(start_x, end_x, steps_x)]
        y_ = [i for i in range(start_y, end_y, steps_y)]
        yaw_ = [i for i in range(start_yaw, end_yaw, steps_yaw)]

        #UAV pose
        uav_x = uav_pose.pose.position.x
        uav_y = uav_pose.pose.position.y
        uav_z = uav_pose.pose.position.z
        uav_angle = euler_from_quaternion([uav_pose.pose.orientation.x,
                                           uav_pose.pose.orientation.y,
                                           uav_pose.pose.orientation.z,
                                           uav_pose.pose.orientation.w])

        #Create new locations to search for the marker board
        waypoints.append(uav_pose)
        for x in x_:
            for y in y_:
                for yaw in yaw_:
                    pose = PoseStamped()
                    pose.pose.position.x = uav_x + x
                    pose.pose.position.y = uav_y + y
                    pose.pose.position.z = uav_z
                    pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(uav_angle[2] + yaw),'rxyz'))
                    waypoints.append(pose)

        return waypoints
    
    """Method used in waypoint generation in GPS2Vision pose estimation test"""
    def generate_waypoints(self, x, y, alt):

        #Initialize waypoints
        waypoints = []
        dis_to_aruco_x = 2
        dx = 0
        dy = 4

        #Initiate waypoints as a grid where the drone faces the aruco marker at all time
        for row in range(7): #Steps in y in meters
            for col in range(9): #Steps in x in meters

                pose = PoseStamped()
                pose.pose.position.x = x+dx
                pose.pose.position.y = y+dy
                pose.pose.position.z = alt

                #To be used so that the drone always facing the marker
                if (y+dy) < 0:
                    yaw = np.deg2rad(90) - np.arctan((dis_to_aruco_x/abs(y+dy)))
                elif (y+dy) > 0:
                    yaw = -1*(np.deg2rad(90) - np.arctan((dis_to_aruco_x/abs(y+dy))))
                else:
                    yaw = 0

                pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))
                dy = dy - 1
                waypoints.append(pose)

            dy = 4
            dx = dx - 1
            dis_to_aruco_x = dis_to_aruco_x + 1

        return waypoints

if __name__ == "__main__":
    gw = waypoint_generator()
