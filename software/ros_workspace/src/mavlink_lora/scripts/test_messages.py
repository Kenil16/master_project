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
This script shows what messages that gets sent though, and how often they arrive. Used to find what messages to filter
and validate the filtering.

It has been tested using a Pixhawk 2.1 flight controller running PX4.

Revision
2018-06-13 FMA First published version
2018-03-14 FMA Cleaned scripts and made them better as examples
'''

# imports
import rospy
from mavlink_lora.msg import mavlink_lora_msg

# parameters
mavlink_lora_sub_topic = '/mavlink_rx'


# defines
class msg:
    def __init__(self, msg_id):
        self.msg_id = msg_id
        self.period = 0
        self.last_heard = rospy.get_time()  # now
        self.total_msg = 0


# variables
msgs_list = []


def update_period(last_time):
    now = rospy.get_time()
    period = now - last_time
    return period


def print_list():
    # clear console
    print('\033[2J')  # clear screen
    print('')
    print('\n\n')
    #print('',)  # go home

    for idx, m,in enumerate(msgs_list):
        print('\033[{2}H{0}: {1:.2f} sec [{3}]').format(m.msg_id, m.period, idx+1, m.total_msg)

    # print total
    print("\n\nTotal Messages: {}").format(len(msgs_list))

    # remove cursor
    print('\033[?25l')  # hide cursor


def sortMsg(item):
    return item.msg_id


def on_mavlink_msg (msg_recived):
    # find in list, or add to list
    for m in msgs_list:
        if m.msg_id == msg_recived.msg_id:
            # update period
            m.period = update_period(m.last_heard)
            m.last_heard = rospy.get_time()
            m.total_msg = m.total_msg + 1
            break
    else:
        # add to list
        newM = msg(msg_recived.msg_id)
        msgs_list.append(newM)

        # sort list msg_id
        msgs_list.sort(key=lambda x: x.msg_id)


# launch node
rospy.init_node('mavlink_lora_msg_transmission_tester')

# ack sub
rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, on_mavlink_msg) # mavlink_msg subscriber

# wait until everything is running
rospy.sleep(1)

# loop until shutdown
rate = rospy.Rate(100)
while not (rospy.is_shutdown()):

    # print list and periods
    print_list()

    rate.sleep()

