#!/usr/bin/env python2

from std_msgs.msg import String, Int8, Float32, Bool
from os import sys
from random import randrange, uniform
from uav_flight_modes import*
from log_data import* 

class transformations_calculations():

    def __init__(self):
        
        self.GPS2Vision_offset = [[0,0], [0,0], [0,0], [0,0]]
        self.uav2aruco_rotation = euler_matrix(0, 0, np.deg2rad(-90), 'rxyz')
        self.publish_local_pose_with_offset = False

    def map_GPS_pose_to_vision(self, aruco_pose, uav_pose):

        #This method maps the GPS pose to that of the aruco board estimate.
        #This insures that the drone follows the same coordinate system when going
        #from GPS to vision which insures a smooth transition between GPS2Vision

        #Get euler angles and rotation matrix
        angle_uav = euler_from_quaternion([uav_pose.pose.orientation.x,
                                           uav_pose.pose.orientation.y,
                                           uav_pose.pose.orientation.z,
                                           uav_pose.pose.orientation.w])

        angle_aruco = euler_from_quaternion([aruco_pose.pose.orientation.x,
                                             aruco_pose.pose.orientation.y,
                                             aruco_pose.pose.orientation.z,
                                             aruco_pose.pose.orientation.w])

        r_uav = quaternion_matrix([uav_pose.pose.orientation.x,
                                   uav_pose.pose.orientation.y,
                                   uav_pose.pose.orientation.z,
                                   uav_pose.pose.orientation.w])


        r_aruco = quaternion_matrix([aruco_pose.pose.orientation.x,
                                     aruco_pose.pose.orientation.y,
                                     aruco_pose.pose.orientation.z,
                                     aruco_pose.pose.orientation.w])

        #Find the rotation between the aruco board and that of the drone to map 
        #vision to the original coordinate system used by the drone 
        self.uav2aruco_rotation = np.matmul(r_aruco.T, r_uav)
        t = np.array([aruco_pose.pose.position.x,
                      aruco_pose.pose.position.y,
                      aruco_pose.pose.position.z, 1])
        t = np.dot(self.uav2aruco_rotation,t)

        #Initialize offset between original drone pose and aruco board 
        self.GPS2Vision_offset = [[uav_pose.pose.position.x, t[0]],
                                  [uav_pose.pose.position.y, t[1]],
                                  [uav_pose.pose.position.z, t[2]],
                                  [angle_uav[2], angle_aruco[2]]]

        #Begin publish estimated aruco board pose with uav offsets
        self.publish_local_pose_with_offset = True

    def calculate_GPS2Vision_offset(self, GPS2Vision_offset, aruco_pose, calculate_setpoint=False):

        #Get current aruco orientation 
        angle = euler_from_quaternion([aruco_pose.pose.orientation.x,
                                       aruco_pose.pose.orientation.y,
                                       aruco_pose.pose.orientation.z,
                                       aruco_pose.pose.orientation.w])

        #Get new yaw from the GPS2Vision offset and aruco yaw estimate 
        yaw = (GPS2Vision_offset[3][0] - (GPS2Vision_offset[3][1] - angle[2]))

        #Calculate the aruco pos relative to the original GPS coordinate system
        t = np.array([aruco_pose.pose.position.x,
                      aruco_pose.pose.position.y,
                      aruco_pose.pose.position.z, 1])
        t = np.dot(self.uav2aruco_rotation,t)

        #Get new transformed pose using GPS2Vision offset and current aruco pos estimate
        pose = PoseStamped()
        pose.pose.position.x = GPS2Vision_offset[0][0] - (GPS2Vision_offset[0][1] - t[0])
        pose.pose.position.y = GPS2Vision_offset[1][0] - (GPS2Vision_offset[1][1] - t[1])
        pose.pose.position.z = GPS2Vision_offset[2][0] - (GPS2Vision_offset[2][1] - t[2])

        if calculate_setpoint:
            pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw,'rxyz'))
        else:
            pose.pose.orientation = Quaternion(*quaternion_from_euler(angle[0], angle[1], yaw,'rxyz'))

        #print(euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]))
        return pose

if __name__ == "__main__":
    tc = transformations_calculations()
