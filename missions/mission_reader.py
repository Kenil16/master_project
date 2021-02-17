#!/usr/bin/env python

import numpy as np
import mavros.setpoint as mavSP
from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
from geometry_msgs.msg import Quaternion, PoseStamped

class missionReader(): 
    
    def __init__(self, fileName):
        self.mission = self.read_mission(fileName)
        self.next_waypoint = PoseStamped()
    
    def update_mission(self):

        param = ' ' 
        param_value = 0
        timeout = 0
        transform = 0

        if not self.mission[0][0] == '-':
            param = self.mission[0][0]
            
            try:
                param_value = int(self.mission[0][1])
            except ValueError:
                param_value = float(self.mission[0][1])
            
            timeout = int(self.mission[0][2])
            
            #Update parameters
            #self.flight_mode.set_param(param, param_value, timeout)
        
        if not self.mission[0][3] == '-':

            self.next_waypoint.pose.position.x = float(self.mission[0][3])
            self.next_waypoint.pose.position.y = float(self.mission[0][4])
            self.next_waypoint.pose.position.z = float(self.mission[0][5])

            angle = Quaternion(*quaternion_from_euler(
                np.deg2rad(float(self.mission[0][6])),
                np.deg2rad(float(self.mission[0][7])),
                np.deg2rad(float(self.mission[0][8]))))

            self.next_waypoint.pose.orientation = angle

        if not self.mission[0][9] == '-':
            transform = int(self.mission[0][9])
            #self.pub_aruco_board.publish(transform)

        self.mission.pop(0)
        print(param + ' ' + str(param_value) + ' ' + str(timeout) + ' ' + str(self.next_waypoint.pose.position.x) + ' ' + str(transform))

    def read_mission(self, file_name):
        
        txtFile = open(file_name,'r')
        data = txtFile.readlines()
        mission = []

        for x in data:
            x = x.split(' ')
            x.pop(-1)
            mission.append(x)
        
        return mission


if __name__ == "__main__":
    mr = missionReader('mission1.txt')
    while len(mr.mission):
        mr.update_mission()
