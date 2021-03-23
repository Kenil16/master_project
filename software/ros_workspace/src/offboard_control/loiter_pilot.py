#!/usr/bin/env python

import rospy
from math import pi, radians, degrees, tan, sin, cos

from std_msgs.msg import (String, Int8, Float64)
from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
from geometry_msgs.msg import Quaternion, PoseStamped

class loiter_pilot(): 
    def __init__(self):

        #Init ROS
        rospy.init_node('loiter_pilot')
        self.rate = rospy.Rate(20)

        #Variables
        self.enable = False
        self.loiterPos = PoseStamped()
        self.curPos = PoseStamped()

        #Subscribers
        rospy.Subscriber('/onboard/state', String, self.onStateChange)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.onPositionChange)
        rospy.Subscriber('/gcs/command', Int8, self.onCommand)

        #Publishers
        self.pub_loiter_setpoint = rospy.Publisher('/onboard/setpoint/loiter_pilot', PoseStamped, queue_size=5)
      
        rospy.loginfo('Loiter: Loiter pilot Ready')

    def _pubMsg(self, msg, topic):
        msg.header.frame_id = "att_pose_loiter"
        msg.header.stamp = rospy.Time.now()
        topic.publish(msg)
        self.rate.sleep()

    def adjustYaw(self, angle=5.0): 
        (_, _, yaw) = euler_from_quaternion(
            [self.loiterPos.pose.orientation.x, 
            self.loiterPos.pose.orientation.y, 
            self.loiterPos.pose.orientation.z, 
            self.loiterPos.pose.orientation.w])

        yaw += radians(angle)
        orientAdj = quaternion_from_euler(0, 0, yaw)

        self.loiterPos.pose.orientation = Quaternion(*orientAdj)
        
    def onCommand(self, msg):
        
        command = str(chr(msg.data))
        command.lower()
        if self.enable:
            if command == 'w':
                self.loiterPos.pose.position.x += 0.5 
            if command == 's':
                self.loiterPos.pose.position.x -= 0.5
            if command == 'a':
                self.loiterPos.pose.position.y += 0.5
            if command == 'd':
                self.loiterPos.pose.position.y -= 0.5    
            if command == 'z':
                self.loiterPos.pose.position.z += 0.5 
            if command == 'x':
                self.loiterPos.pose.position.z -= 0.5
            if command == 'q':
                self.adjustYaw(5.0)
            if command == 'e':
                self.adjustYaw(-5.0)
        else:
            options = "wasdqezx" # options as above
            if command in options: 
                rospy.logwarn('Loiter: Pilot not enabled') 
                
    def onStateChange(self, msg):
        if msg.data == 'loiter':
            rospy.loginfo('Loiter: Enabled')
            self.loiterPos = self.curPos
            self.enable = True
        else:
            if self.enable:
                rospy.loginfo('Loiter: Disabled')
                self.enable = False
        
    def onPositionChange(self,msg):
        self.curPos = msg

    def run(self):
        while not rospy.is_shutdown():
            if self.enable:
                self._pubMsg(self.loiterPos, self.pub_loiter_setpoint)
                self.rate.sleep()

if __name__ == "__main__":
    node = loiter_pilot()
    node.run()
