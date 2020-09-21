#!/usr/bin/env python
# /***************************************************************************
# MavLink LoRa node (ROS) clear mission example script
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
# ****************************************************************************
'''
This example script shows how to clear a mission on a drone.

It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2018-06-13 FMA First published version
2018-03-14 FMA Cleaned scripts and made them better as examples
'''
# parameters
mavlink_lora_mission_start_topic = 'mavlink_interface/command/start_mission'

# imports
import rospy
from mavlink_lora.msg import mavlink_lora_command_start_mission, mavlink_lora_mission_ack


def on_ack_received_callback(msg):
    # result
    print(msg.result_text)

    # shut node down
    rospy.signal_shutdown("User quit")


# variables
target_sys = 0  # reset by first message
target_comp = 0

# launch node
rospy.init_node('mavlink_lora_mission_start')

# pubs
mavlink_msg_pub = rospy.Publisher(mavlink_lora_mission_start_topic, mavlink_lora_command_start_mission, queue_size=0)

# subs
mavlink_mission_ack_sub = rospy.Subscriber("mavlink_interface/mission/ack", mavlink_lora_mission_ack,
                                           on_ack_received_callback)

# wait until everything is running
rospy.sleep(1)

# send command
msg = mavlink_lora_command_start_mission()
msg.first_item = 0
mavlink_msg_pub.publish(msg)

# loop until shutdown
while not (rospy.is_shutdown()):
    # do stuff
    rospy.sleep(5)
