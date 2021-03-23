#!/usr/bin/env python2

from geometry_msgs.msg import PoseStamped, Quaternion

class read_missions():

    def __init__(self):
        self.mission = []
        self.next_waypoint = PoseStamped()
        self.next_max_pose_error = 0.0

    def update_mission(self):

        self.next_waypoint.pose.position.x = float(self.mission[0][0])
        self.next_waypoint.pose.position.y = float(self.mission[0][1])
        self.next_waypoint.pose.position.z = float(self.mission[0][2])

        self.next_waypoint.pose.orientation = Quaternion(*quaternion_from_euler(np.deg2rad(float(self.mission[0][3])),
                                                                                np.deg2rad(float(self.mission[0][4])),
                                                                                np.deg2rad(float(self.mission[0][5]))))

        if not self.mission[0][6] == '-':
            self.waypoint_check_pose_error = float(self.mission[0][6])

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
    rm = read_missions() 
