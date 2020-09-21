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
This example script shows how to precision land where the drone currently is.

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
mavlink_lora_land_pub_topic = '/mavlink_interface/command/land'

# imports
import rospy
from mavlink_lora.msg import mavlink_lora_command_ack, mavlink_lora_pos, mavlink_lora_command_land

# variables
target_sys = 0  # reset by first message
target_comp = 0
current_lat = 0
current_lon = 0
current_alt = 0
pos_received = False
land_sent = False


def on_mavlink_msg (msg):
    # print ack msg
    print(msg)


def on_gps_global_pos(msg):
    global pos_received, current_lon, current_lat, current_alt

    # only set it once
    if pos_received:
        return

    current_lat = msg.lat
    current_lon = msg.lon
    current_alt = msg.alt

    pos_received = True

    print("Received global pos")


def send_mavlink_land():
    global current_alt, current_lat, current_lon

    # make msg with curr coordinates to make it land at current position
    msg = mavlink_lora_command_land()
    msg.lat = current_lat
    msg.lon = current_lon
    msg.altitude = 10  # make it go to 10m altitude before starting landing. Can be used for precision landing systems
    msg.yaw_angle = float('NaN')  # unchanged angle
    msg.abort_alt = 5
    msg.precision_land_mode = 0  # 2=required precision landing, 1= opportunistic precision land, 0=gps landing

    mavlink_land_pub.publish(msg)


# launch node
rospy.init_node('mavlink_lora_land')
mavlink_land_pub = rospy.Publisher(mavlink_lora_land_pub_topic, mavlink_lora_command_land, queue_size=0)

# ack sub
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
        print("land sent")
        send_mavlink_land()

        land_sent = True
