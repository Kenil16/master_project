#!/usr/bin/env python
#/***************************************************************************
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
#****************************************************************************
'''
This example script shows how to obtain basic status of a UAV.

NOT YET: It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2018-05-29 KJ First version
'''
# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
mavlink_lora_status_sub_topic = '/mavlink_status'
mavlink_lora_pos_sub_topic = '/mavlink_pos'
mavlink_lora_atti_sub_topic = '/mavlink_attitude'
mavlink_lora_keypress_sub_topic = '/keypress'
update_interval = 10

# imports
import rospy
import struct
from std_msgs.msg import Int8
from mavlink_lora.msg import mavlink_lora_msg, mavlink_lora_status, mavlink_lora_pos, mavlink_lora_attitude
from math import pi, sqrt, sin, cos, atan2

# defines
R = 6371000 # Assumed Earth radius in meter
DEG2RAD = pi/180.0
RAD2DEG = 180.0/pi

class gcs_node:
	def __init__(self):
		self.msg = mavlink_lora_msg()
		self.request_sent = False

		# status variables
		self.batt_volt = 0.0
		self.last_heard = 0
		self.last_heard_sys_status = 0
		self.lat = 0.0
		self.lon = 0.0
		self.alt = 0.0
		self.home_lat = 0.0
		self.home_lon = 0.0
		self.yaw = 0.0
		self.pitch = 0.0
		self.roll = 0.0

		# launch node
		rospy.init_node('mavlink_lora_gcs_simple', disable_signals = True)
		self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0)
		rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, self.on_mavlink_msg)
		rospy.Subscriber(mavlink_lora_status_sub_topic, mavlink_lora_status, self.on_mavlink_lora_status)
		rospy.Subscriber(mavlink_lora_pos_sub_topic, mavlink_lora_pos, self.on_mavlink_lora_pos)
		rospy.Subscriber(mavlink_lora_atti_sub_topic, mavlink_lora_attitude, self.on_mavlink_lora_attitude)
		rospy.Subscriber(mavlink_lora_keypress_sub_topic, Int8, self.on_keypress)
		self.rate = rospy.Rate(update_interval)
		rospy.sleep (1) # wait until everything is running

	def gcd_haversine (self, lat1, lon1, lat2, lon2):
		lat1 *= DEG2RAD	
		lon1 *= DEG2RAD	
		lat2 *= DEG2RAD	
		lon2 *= DEG2RAD	
		dlat = lat2-lat1
		dlon = lon2-lon1
		a = sin(dlat/2.)**2 + sin(dlon/2.)**2 * cos(lat1) * cos(lat2)
		c = 2 * atan2(sqrt(a), sqrt(1-a))
		return (R * c)

	def update_display (self):
		now = rospy.get_time()
		print '\033[2J' # clear screen
		print '', # go home

		# update last_heard
		t = now - self.last_heard
		if t < 86400:
			last_heard_text = '%ds' % t
		else:
			last_heard_text = 'Never'

		# update last status
		t_sys_status = now - self.last_heard_sys_status
		if t_sys_status < 86400:
			last_heard_status_text = '%ds' % t_sys_status
		else:
			last_heard_status_text = 'Never'

		# update battery status
		if self.batt_volt == 0 or t_sys_status > 60:
			batt_text = 'Unknown'
		else:
			batt_text = '%.1fV' % self.batt_volt

		# update pos status
		if self.lat == 0 and self.lon == 0:
			pos_text = 'Unknown'
		else:
			pos_text = '%02.5f %03.5f' % (self.lat, self.lon)

		# update altitude status
		if self.alt == 0:
			alt_text = 'Unknown'
		else:
			alt_text = '%.1fm ' % (self.alt)

		# update distance status
		if self.home_lat == 0 and self.home_lon == 0:
			if self.lat == 0 and self.lon == 0:
				home_text = 'Unknown'
			else:
				home_text = 'Press h to set home position'
		else:
			home_text = '%.1fm' % self.gcd_haversine(self.lat, self.lon, self.home_lat, self.home_lon)

		# update attitude text
		if self.yaw == 0 and self.pitch == 0 and self.roll == 0:
			atti_text = 'Unknown'
		else:
			atti_text = 'Yaw: %03.1f Pitch: %03.1f Roll: %03.1f' % (self.yaw*180/pi, self.pitch*180/pi, self.roll*180/pi)


		print '\033[1HLast heard:         %s' % last_heard_text
		print '\033[2HLast system status: %s' % last_heard_status_text
		print '\033[4HBattery:            %s' % batt_text
		print '\033[6HPosition:           %s' % pos_text
		print '\033[7HAltitude:           %s' % alt_text  
		print '\033[8HDistance:           %s' % home_text
		print '\033[10HAttitude:           %s' % atti_text
		print '\033[12HPress: h to set home position, q to quit'
		print '\033[?25l' # hide cursor
	
	def on_mavlink_msg (self, msg):
		'''
		if msg.msg_id == MAVLINK_MSG_ID_PARAM_VALUE:
			(param_value, param_count, param_index, param_id, param_type) = struct.unpack('<fHH16sB', msg.payload)	
			print param_id, param_value, param_count
		'''
		pass
	
	def on_mavlink_lora_status (self, msg):
		self.last_heard = msg.last_heard.secs + msg.last_heard.nsecs/1.0e9
		self.last_heard_sys_status = msg.last_heard_sys_status.secs + msg.last_heard_sys_status.nsecs/1.0e9
		self.batt_volt = msg.batt_volt / 1000.0

	def on_mavlink_lora_pos (self, msg):
		self.lat = msg.lat
		self.lon = msg.lon
		self.alt = msg.alt

	def on_mavlink_lora_attitude (self, msg):
		self.yaw = msg.yaw
		self.pitch = msg.pitch
		self.roll = msg.roll

	def on_keypress (self, msg):
		if msg.data == ord('h'):
			if self.lat != 0 or self.lon != 0:
				self.home_lat = self.lat
				self.home_lon = self.lon
		elif msg.data == ord('q'):
			rospy.signal_shutdown('User quit')

	def send_mavlink_param_req_list(self):
		'''
		# no need to set sys_id, comp_id or checksum, this is handled by the mavlink_lora node.
		msg.header.stamp = rospy.Time.now()
		msg.msg_id = MAVLINK_MSG_ID_PARAM_REQUEST_LIST
		msg.payload_len = MAVLINK_MSG_ID_PARAM_REQUEST_LIST_LEN
		msg.payload = struct.pack('<BB', target_system, target_component)
		mavlink_msg_pub.publish(msg)
		'''
		pass

	def run(self):
		# loop until shutdown
		while not (rospy.is_shutdown()):
			# do stuff
			self.update_display()
			if self.request_sent == False:
				print 'Requesting...'
				self.request_sent = True	

			# sleep the defined interval
			self.rate.sleep()

if __name__ == '__main__':
	gcs = gcs_node()
	gcs.run()
	

