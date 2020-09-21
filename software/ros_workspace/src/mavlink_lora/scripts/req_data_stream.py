#!/usr/bin/env python
#/***************************************************************************
# MavLink LoRa node (ROS) request data stream script
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
# (INCLUDING NEGIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
'''
Revision
2018-06-13 KJ First published version
'''
# parameters
target_sys = 66 # target system (1 = Pixhawk2 PX4, 66 = AutoQuad)
target_comp = 0 # target component

# other defines
ros_node_name = 'streams_cfg'
ros_node_update_interval = 10
mavlink_lora_rx_sub = '/mavlink_rx'
mavlink_lora_tx_pub = '/mavlink_tx'

MAVLINK_MSG_ID_REQUEST_DATA_STREAM = 66
MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN = 6

MAVLINK_MSG_ID_DATA_STREAM = 67
MAVLINK_MSG_ID_DATA_STREAM_LEN = 4

# imports
from sys import argv
import rospy
import struct
from mavlink_lora.msg import mavlink_lora_msg

class ros_node():
	def __init__(self, stream_id, stream_rate, stream_start_stop):
		# save variables
		self.stream_id = stream_id
		self.stream_rate = stream_rate
		self.stream_start_stop = stream_start_stop

		# initiate variables
		self.stop = False
		self.target_sys = 0
		self.target_comp = 0
		self.first_msg_ok = False
		self.request_sent = False
		self.msg = mavlink_lora_msg()

		# initiate node
		rospy.init_node(ros_node_name)
		self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_tx_pub, mavlink_lora_msg, queue_size=0)
		rospy.Subscriber(mavlink_lora_rx_sub, mavlink_lora_msg, self.on_mavlink_msg)
		self.rate = rospy.Rate(ros_node_update_interval)

		# wait until everything is running (important)
		rospy.sleep (3)

	def on_mavlink_msg (self, msg):
		if self.first_msg_ok == False:
			self.first_msg_ok = True
			self.target_sys = msg.sys_id

		if msg.msg_id == MAVLINK_MSG_ID_DATA_STREAM:
			(stream_rate, stream_id, stream_on_off) = self.unpack_data_stream(msg.payload)
			text = "Received stream id %d at rate %d " % (stream_id, stream_rate)
			if stream_on_off == 1:
				text = text + 'ON'
			else:
				text = text + 'OFF'
			print (text)
			self.stop = True

	def send_mavlink_request_data_stream(self):
		self.msg.header.stamp = rospy.Time.now()
		self.pack_request_data_stream(self.stream_id, self.stream_rate, self.stream_start_stop)
		self.mavlink_msg_pub.publish(self.msg)

	def pack_request_data_stream(self, stream_id, stream_rate, start_stop):
		self.msg.msg_id = MAVLINK_MSG_ID_REQUEST_DATA_STREAM
		self.msg.payload_len = MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN
		self.msg.payload = struct.pack('<HBBBB',  stream_rate, self.target_sys, self.target_comp, stream_id, start_stop)

	def unpack_data_stream(self, payload):
		(stream_rate, stream_id, stream_on_off) = struct.unpack('<HBB', payload)
		return (stream_rate, stream_id, stream_on_off)

	def loop(self):
		while not (rospy.is_shutdown() or self.stop):
			# do stuff
			if self.request_sent == False and self.first_msg_ok == True:
				self.send_mavlink_request_data_stream()
				self.request_sent = True	

			# sleep the defined interval
			self.rate.sleep()

if __name__ == "__main__":
	if len(argv) == 1:
		print ("Usage: request_stream.py [stream_id] [rate] [start/stop]")

		print ("steam_id")
		print ("  0  MAV_DATA_STREAM_ALL")
		#print ("  (all data streams)")
		print ("  1  MAV_DATA_STREAM_RAW_SENSORS")
		#print ("  (IMU_RAW, GPS_RAW, GPS_STATUS)")
		print ("  2  MAV_DATA_STREAM_EXTENDED_STATUS")
		#print ("  (GPS_STATUS, CONTROL_STATUS, AUX_STATUS)")
		print ("  3  MAV_DATA_STREAM_RC_CHANNELS")
		#print ("  (RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW)")
		print ("  4  MAV_DATA_STREAM_RAW_CONTROLLER")
		#print ("  (ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT)")
		print ("  6  MAV_DATA_STREAM_POSITION")
		#print ("  (LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT)")

		print ("")
		print ("rate")
		print ("  0-200 Hz (AutoQuad limit)")
		print ("")
		print ("start/stop")
		print ("  0  Stop sending")
		print ("  1  Start sending")
	else:
		stream_id = int(argv[1])
		stream_rate = int(argv[2])
		stream_start_stop = int(argv[3])
		print("Requesting stream id %d at rate %d start/stop %d" % (stream_id, stream_rate, stream_start_stop))
		rn = ros_node(stream_id, stream_rate, stream_start_stop)
		rn.loop()

