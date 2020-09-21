#!/usr/bin/env python
#/***************************************************************************
# MavLink LoRa node (ROS) param list example script
# Copyright (c) 2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
'''
This example script shows how to set mode on a drone.

It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2018-06-13 FMA First published version
2018-03-14 FMA Cleaned scripts and made them better as examples
'''

# parameters
mavlink_lora_cmd_ack_topic = '/mavlink_interface/command/ack'
mavlink_lora_setpoint_pub_topic = '/mavlink_interface/command/set_target_position_local_ned'

# imports
from sys import argv
import rospy
from mavlink_lora.msg import mavlink_lora_command_ack, mavlink_lora_command_takeoff, mavlink_lora_enable_offboard, mavlink_lora_set_position_target_local_ned

# variables
target_sys = 0 # reset by first message
target_comp = 0


def on_mavlink_msg (msg):
    # print cmd ack
    print(msg)


# launch node
rospy.init_node('mavlink_lora_set_local_target')
mavlink_setpoint_pub = rospy.Publisher(mavlink_lora_setpoint_pub_topic, mavlink_lora_set_position_target_local_ned, queue_size=0)

# ack sub
rospy.Subscriber(mavlink_lora_cmd_ack_topic, mavlink_lora_command_ack, on_mavlink_msg) # mavlink_msg subscriber

# wait until everything is running
rospy.sleep(1)

# make setpoint
target = mavlink_lora_set_position_target_local_ned()

if len(argv) == 4: # remember it counts the python file to run too
    target.x = float(argv[1])
    target.y = float(argv[2])
    target.z = float(argv[3])
else:
    print("Usage: set_offboard_setpoint.py [x] [y] [z]")
    print("0,0,0 is the position of the drone when it boots. Coordinates is given in LOCAL_NED frame")
    print("----------------------------")
    print("As no correct input was given the setpoint is set to 0,0,-5")
    target.x = 0
    target.y = 0
    target.z = -5

mavlink_setpoint_pub.publish(target)

# sleep enough to get ack
rospy.sleep(5)
