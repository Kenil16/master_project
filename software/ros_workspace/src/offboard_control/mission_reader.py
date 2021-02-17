#!/usr/bin/env python

import numpy as np
import mavros.setpoint as mavSP
from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
from geometry_msgs.msg import Quaternion

class missionReader(): 
    def __init__(self, fileName):
        self.nextWaypoint = mavSP.PoseStamped()
        self.nextState = 'idle'
        self.nextSubState = 'idle'
        self.nextCommand = 't'
        self.nextParam = ' '
        self.nextParamVal = 0
        self.missionState = 'idle'

        self.index = 0

        self.mission = self.readMission(fileName)

    def getNext(self, forward):
        if forward == False:
            self.index = self.index - 1

        self.nextWaypoint.pose.position.x = float(self.mission[self.index][0])
        self.nextWaypoint.pose.position.y = float(self.mission[self.index][1])
        self.nextWaypoint.pose.position.z = float(self.mission[self.index][2])

        angle = Quaternion(*quaternion_from_euler(
            np.deg2rad(float(self.mission[self.index][3])),
            np.deg2rad(float(self.mission[self.index][4])),
            np.deg2rad(float(self.mission[self.index][5]))))

        self.nextWaypoint.pose.orientation = angle

        self.nextState    = self.mission[self.index][6]
        self.nextSubState = self.mission[self.index][7]
        self.nextCommand  = self.mission[self.index][8]
        self.nextParam    = self.mission[self.index][9]
        self.nextParamVal = self.mission[self.index][10]
        self.missionState = self.mission[self.index][11]

        self.index = self.index + 1
        if self.missionState == 'complete':
            self.index = 0

    def readMission(self, fileName):
        fileName = '../mission_files/' + fileName

        txtFile = open(fileName,'r')
        data = txtFile.readlines()
        arr = []

        nextPos = [0.0,0.0,0.0]
        nextOri = [0.0,0.0,0.0]
        nextState = 'idle'
        nextSubState = 'idle'
        nextCommand = 't'
        nextParam = ' '
        nextParamVal = 0
        missionState = 'idle'

        for x in data:
            tmp = x.split(';')
            if tmp[0] == 'COMMAND':
                nextCommand = tmp[1]
                nextState = tmp[2].split('\n')[0]
                missionState = 'sendCommand'
            if tmp[0] == 'STATE':
                nextState = tmp[1].split('\n')[0]
                missionState = 'setState'
            if tmp[0] == 'SUBSTATE':
                nextSubState = tmp[1].split('\n')[0]
                missionState = 'setSubState'
            if tmp[0] == 'WAYPOINT':                
                nextPos[0] = float(tmp[1])
                nextPos[1] = float(tmp[2])
                nextPos[2] = float(tmp[3])
                nextOri[0] = float(tmp[4])
                nextOri[1] = float(tmp[5])
                nextOri[2] = float(tmp[6].split('\n')[0])
                missionState = 'setWaypoint'
            if tmp[0] == 'PARAM':
                nextParam = tmp[1]
                nextParamVal = float(tmp[2].split('\n')[0])
                missionState = 'updateParam'
            if tmp[0] == 'COMPLETE\n':
                missionState = 'complete'

            arr.append([nextPos[0],
                nextPos[1],
                nextPos[2],
                nextOri[0],
                nextOri[1],
                nextOri[2],
                nextState,
                nextSubState,
                nextCommand,
                nextParam,
                nextParamVal,
                missionState])

        return np.array(arr)




if __name__ == "__main__":
    mr = missionReader('../mission_files/mission_00.txt')
    for x in range(22):
        mr.getNext(True)

    print(mr.missionState)
    




