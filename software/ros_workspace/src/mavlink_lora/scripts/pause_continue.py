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
This example script shows how to pause or continue a mission on a drone.

Currently unsupported on PX4

Revision
2018-06-13 FMA First published version
2018-03-14 FMA Cleaned scripts and made them better as examples
'''

# parameters
mavlink_lora_cmd_ack_topic = '/mavlink_interface/command/ack'
mavlink_lora_pause_continue_pub_topic = 'mavlink_interface/command/pause_continue'

# imports
import rospy
from sys import argv
from mavlink_lora.msg import mavlink_lora_command_ack
from std_msgs.msg import Bool

# variables
global current_lat, current_lon, current_alt, pos_received, land_sent

target_sys = 0 # reset by first message
target_comp = 0
pause_sent = False


def on_mavlink_msg (msg):
    # print cmd ack
    print(msg)


def send_mavlink_pause_continue(hold):
    # make msg
    msg = Bool()
    msg.data = int(hold)

    mavlink_pause_hold_pub.publish(msg)


# check input argv
if len(argv) == 1:
    print("Usage: pause_continue.py [continue/pause]")
    print("Sending 0 means pause and hold current position, 1 means continue mission")

# save what option we took
sendText = ""
if argv[1] == True:
    sendText = "sent mission continue"
else:
    sendText = "sent mission pause. Holding current position"

# launch node
rospy.init_node('mavlink_lora_pause_continue')
mavlink_pause_hold_pub = rospy.Publisher(mavlink_lora_pause_continue_pub_topic, Bool, queue_size=0)

# ack sub
rospy.Subscriber(mavlink_lora_cmd_ack_topic, mavlink_lora_command_ack, on_mavlink_msg)

# wait until everything is running
rospy.sleep(1)

# loop until shutdown
while not (rospy.is_shutdown()):
    if not pause_sent:
        print(sendText)
        send_mavlink_pause_continue(argv[1])
        pause_sent = True

    rospy.sleep(5)
