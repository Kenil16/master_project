#!/usr/bin/python

"""
relay_swarm.py:

"""

###############################################
# Standard Imports                            #
###############################################
import time
import threading
from math import *

###############################################
# ROS Imports                                 #
###############################################
import rospy
import rospkg

###############################################
# ROS Topic messages                          #
###############################################
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

###############################################
# Offboad Control class                       #
###############################################
class VRPNPoseRelay:
    def __init__(self, *args):
        rospy.init_node('relay_server')

        self.sent = 0

        # Publishers
        self.pub_mocap = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

        # Subscribers
        #self.sub_vrpn_pose = rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb_vrpn_pose)
        self.sub_aruco_pose = rospy.Subscriber("/aruco_marker_pose", PoseStamped, self.cb_aruco_pose)

        # Spin until the node is stopped
        rospy.spin()

    """
    Callbacks
    * cb_vrpn_pose
    """
    def cb_vrpn_pose(self,data):
        tmp_c = rospy.get_namespace()
        tmp_c = tmp_c.replace('/', 'sdu_drone_mono_cam')
        #print("ns: {}".format(tmp_c))

        #print("length --> {}".format(len(data.name)))

        for x in range(0, len(data.name)):
            #print("ns: {}".format(data.name[x]))

            if(data.name[x] == tmp_c):
                tmp = PoseStamped()

                tmp.header.stamp = rospy.Time.now()
                tmp.header.frame_id = "world"
                tmp.header.seq = self.sent

                tmp.pose = data.pose[x]

                self.sent = self.sent + 1

                self.pub_mocap.publish(tmp)

    def cb_aruco_pose(self,data):
        
        tmp = PoseStamped()
        tmp.header.stamp = rospy.Time.now()
        tmp.header.frame_id = "world"
        tmp.header.seq = self.sent
        tmp.pose = data.pose
        self.sent = self.sent + 1
        self.pub_mocap.publish(tmp)

if __name__ == '__main__':
    VRPNPR = VRPNPoseRelay()

