#!/usr/bin/env python
#/***************************************************************************
# MavLink LoRa node (ROS) mission library
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
Documentation: http://mavlink.org/messages/common

Revision
2018-06-13 KJ First published version
'''

# defines
MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST = 37
MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN = 6

MAVLINK_MSG_ID_MISSION_ITEM = 39

MAVLINK_MSG_ID_MISSION_REQUEST = 40
MAVLINK_MSG_ID_MISSION_REQUEST_LEN = 4

MAVLINK_MSG_ID_MISSION_SET_CURRENT = 41
MAVLINK_MSG_ID_MISSION_SET_CURRENT_LEN = 4

MAVLINK_MSG_ID_MISSION_CURRENT = 42
MAVLINK_MSG_ID_MISSION_CURRENT_LEN = 2

MAVLINK_MSG_ID_MISSION_REQUEST_LIST = 43
MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN = 2

MAVLINK_MSG_ID_MISSION_COUNT = 44
MAVLINK_MSG_ID_MISSION_COUNT_LEN = 4

MAVLINK_MSG_ID_MISSION_CLEAR_ALL = 45
MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN = 2

MAVLINK_MSG_ID_MISSION_ACK = 47
MAVLINK_MSG_ID_MISSION_ACK_LEN = 3

import struct
from mavlink_lora.msg import mavlink_lora_msg

class mission_lib():
	def __init__(self):
		self.msg = mavlink_lora_msg()

	def set_target (self, target_sys, target_comp):
		self.target_sys = target_sys
		self.target_comp = target_comp

	def pack_mission_set_current(self, mission_id):
		self.msg.msg_id = MAVLINK_MSG_ID_MISSION_SET_CURRENT
		self.msg.payload_len = MAVLINK_MSG_ID_MISSION_SET_CURRENT_LEN
		self.msg.payload = struct.pack('<HBB', mission_id, self.target_sys, self.target_comp)

	def pack_mission_req_list(self):
		self.msg.msg_id = MAVLINK_MSG_ID_MISSION_REQUEST_LIST
		self.msg.payload_len = MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN
		self.msg.payload = struct.pack('<BB', self.target_sys, self.target_comp)

	def pack_mission_req(self, mission_id):
		self.msg.msg_id = MAVLINK_MSG_ID_MISSION_REQUEST
		self.msg.payload_len = MAVLINK_MSG_ID_MISSION_REQUEST_LEN
		self.msg.payload = struct.pack('<HBB', mission_id, self.target_sys, self.target_comp)

	def unpack_mission_count(self, payload):
		(count, target_sys, target_comp) = struct.unpack('<HBB', payload)
		return count

	def unpack_mission_current(self, payload):
		current = struct.unpack('<H', payload)
		return current
		
	def unpack_mission_item(self, payload):
		(param1, param2, param3, param4, x, y, z, seq, cmd, target_sys, target_comp, frame, cur, autocont) = struct.unpack('fffffffHHBBBBB', payload)
		return [param1, param2, param3, param4, x, y, z, seq, cmd, cur, autocont]
	
if __name__ == "__main__":
	pass

