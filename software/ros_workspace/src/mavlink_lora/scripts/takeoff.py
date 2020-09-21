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
This example script shows how to send takeoff to a drone.

It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2018-06-13 FMA First published version
2018-03-14 FMA Cleaned scripts and made them better as examples
'''

# parameters
mavlink_lora_cmd_ack_topic = '/mavlink_interface/command/ack'
mavlink_lora_pos_topic = '/mavlink_pos'
mavlink_lora_arm_pub_topic = '/mavlink_interface/command/arm_disarm'
mavlink_lora_takeoff_pub_topic = '/mavlink_interface/command/takeoff'

# imports
import rospy
from mavlink_lora.msg import mavlink_lora_command_ack, mavlink_lora_command_takeoff, mavlink_lora_pos

# variables
global current_lat, current_lon, current_alt, pos_received, land_sent

target_sys = 0 # reset by first message
target_comp = 0
current_lat = 0 #55.057883
current_lon = 0 #10.569678
current_alt = 0
pos_received = False
land_sent = False


def on_mavlink_msg (msg):
    # print cmd ack
    print(msg)


def on_gps_global_pos(msg):
    global pos_received, current_lon, current_lat, current_alt

    if pos_received:
        return
    current_lat = msg.lat
    current_lon = msg.lon
    current_alt = msg.alt

    pos_received = True

    print("Received global pos")


def send_mavlink_takeoff(alt):

    global current_alt, current_lat, current_lon

    msg = mavlink_lora_command_takeoff()
    msg.lat = current_lat
    msg.lon = current_lon
    msg.alt = alt
    msg.yaw_angle = float('NaN') # unchanged angle
    msg.pitch = 0

    mavlink_takeoff_pub.publish(msg)


# launch node
rospy.init_node('mavlink_lora_takeoff')

# pubs
mavlink_takeoff_pub = rospy.Publisher(mavlink_lora_takeoff_pub_topic, mavlink_lora_command_takeoff, queue_size=0)

# ack and pos sub
rospy.Subscriber(mavlink_lora_cmd_ack_topic, mavlink_lora_command_ack, on_mavlink_msg)
rospy.Subscriber(mavlink_lora_pos_topic, mavlink_lora_pos, on_gps_global_pos)

# wait until everything is running
rospy.sleep(1)

# loop until shutdown
print("waiting on current gps coordinate")
while not (rospy.is_shutdown()):

    # do stuff
    global pos_received, land_sent

    if pos_received and land_sent == False:
        print("takeoff sent")
        send_mavlink_takeoff(10)

        land_sent = True
