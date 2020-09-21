#!/usr/bin/env python
# /***************************************************************************
# MavLink LoRa node (ROS) example script
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
This example script shows how to limit default messages published from the PX4 UAV.

NOT YET: It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2019-09-16 FMA First version
'''

# imports
import rospy
import threading
from mavlink_lora.msg import mavlink_lora_command_message_interval, mavlink_lora_command_ack

# defines
MAV_RESULT_ACCEPTED = 0
MAVLINK_MSG_ID_SYSTEM_TIME = 2
MAVLINK_MSG_ID_SYSTEM_STATUS = 1
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_ATTITUDE_QUATERNION = 31
MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32
MAVLINK_MSG_ID_RC_CHANNELS_SCALED = 34      # kept failing
MAVLINK_MSG_ID_RC_CHANNELS_RAW = 35         # kept failing
MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = 36
MAVLINK_MSG_ID_RC_CHANNELS = 65
MAVLINK_MSG_ID_MANUAL_CONTROL = 69
MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE = 70    # kept failing
MAVLINK_MSG_ID_VFR_HUD = 74
MAVLINK_MSG_ID_ATTITUDE_TARGET = 83
MAVLINK_MSG_ID_HIGHRES_IMU = 105
MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET = 140
MAVLINK_MSG_ID_ALTITUDE = 141
MAVLINK_MSG_ID_BATTERY_STATUS = 147         # kept failing
MAVLINK_MSG_ID_ESTIMATOR_STATUS = 230       # kept failing
MAVLINK_MSG_ID_VIBRATION = 241
MAVLINK_MSG_ID_EXTENDED_SYS_STATE = 245


class message_interval_node:
    def __init__(self):

        # vars
        self.message_index = 0
        # -1 disables the message, while any other positive integer defines the frequency
        self.message_array = [[MAVLINK_MSG_ID_SYSTEM_TIME, 1], [MAVLINK_MSG_ID_ATTITUDE, -1],
                              [MAVLINK_MSG_ID_ATTITUDE_QUATERNION, -1], [MAVLINK_MSG_ID_LOCAL_POSITION_NED, -1],
                              [MAVLINK_MSG_ID_RC_CHANNELS_SCALED, -1], [MAVLINK_MSG_ID_RC_CHANNELS_RAW, -1],
                              [MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, -1], [MAVLINK_MSG_ID_RC_CHANNELS, -1],
                              [MAVLINK_MSG_ID_MANUAL_CONTROL, -1], [MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, -1],
                              [MAVLINK_MSG_ID_VFR_HUD, -1], [MAVLINK_MSG_ID_ATTITUDE_TARGET, -1],
                              [MAVLINK_MSG_ID_HIGHRES_IMU, -1], [MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET, -1],
                              [MAVLINK_MSG_ID_ALTITUDE, 1], [MAVLINK_MSG_ID_BATTERY_STATUS, 1],
                              [MAVLINK_MSG_ID_ESTIMATOR_STATUS, -1], [MAVLINK_MSG_ID_VIBRATION, 1],
                              [MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 1]]


        self.limit_tries = 0

        # launch node
        rospy.init_node('mavlink_lora_message_interval')

        # subs
        rospy.Subscriber("mavlink_interface/command/ack", mavlink_lora_command_ack, self.command_ack_callback)

        # pubs
        self.message_interval_pub = rospy.Publisher("mavlink_interface/command/message_interval",
                                                    mavlink_lora_command_message_interval, queue_size=1)

        # rate to run the loop
        self.rate = rospy.Rate(10)

        # wait until everything is running
        rospy.sleep(1)

    def run(self):
        # start the transmission
        self.send_limit()

        # loop until shutdown
        while not (rospy.is_shutdown()):
            # sleep the defined interval
            self.rate.sleep()

    def send_limit(self):
        # If the array of messages has not been run through
        if self.message_index < len(self.message_array):
            # Send message interval
            msg = mavlink_lora_command_message_interval()
            msg.message_id, msg.interval = self.message_array[self.message_index][0], \
                                           self.message_array[self.message_index][1]
            self.message_interval_pub.publish(msg)

        # Else, go to next state
        else:
            # shutdown rospy
            rospy.loginfo("Finished")
            rospy.signal_shutdown("User quit")

    # Callbacks
    def command_ack_callback(self, msg):
        # If command is rejected, wait 2 seconds and try to send again
        if msg.result != MAV_RESULT_ACCEPTED and self.limit_tries <= 5:
            # Check what command it was, and resend it
            rospy.loginfo("Retrying in 1 seconds")
            self.limit_tries += 1
            self.resend_timer = threading.Timer(1.0, self.send_limit)
            self.resend_timer.start()

        elif self.limit_tries > 5:
            rospy.loginfo("Maximum limit tries reached. Proceeding.")
            self.message_index += 1
            self.limit_tries = 0
            self.send_limit()

        # Else, if mission accepted, move to next command or state
        elif msg.result == MAV_RESULT_ACCEPTED:
            if self.message_array[self.message_index][1] == -1:
                rospy.loginfo("Message ID " + str(self.message_array[self.message_index][0]) + " disabled")
            else:
                rospy.loginfo("Message ID " + str(self.message_array[self.message_index][0]) + " limited to " + str(
                    self.message_array[self.message_index][1]) + " Hz")
            self.message_index += 1
            # send next limit
            self.send_limit()



if __name__ == '__main__':
    gcs = message_interval_node()
    gcs.run()
