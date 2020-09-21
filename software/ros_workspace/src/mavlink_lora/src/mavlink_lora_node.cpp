/***************************************************************************
# MavLink LoRa node (ROS)
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

Revision
2018-04-17 KJ First released test version
2018-05-29 KJ Added support for GPS_RAW_INT messages, corrected voltage handling
2018-06-12 KJ Added attitude topic (works with AutoQuad for now), added
              serial lib making it ROS Melodic compatible, added parameters
              for serial dev. and baud.
2019-02-28 FMA Major update and merge with developed functionality
2019-09-16 FMA Redone to use OOP edition of mavlink library
****************************************************************************/
/* includes */
#include <algorithm>    // std::max
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <mavlink_msgs/mavlink_lora_msg.h>
#include <mavlink_msgs/mavlink_lora_pos.h>
#include <mavlink_msgs/mavlink_lora_attitude.h>
#include <mavlink_msgs/mavlink_lora_status.h>
#include <mavlink_msgs/mavlink_lora_mission_item_int.h>
#include <mavlink_msgs/mavlink_lora_mission_list.h>
#include <mavlink_msgs/mavlink_lora_mission_ack.h>
#include <mavlink_msgs/mavlink_lora_command_ack.h>
#include <mavlink_msgs/mavlink_lora_command_start_mission.h>
#include <mavlink_msgs/mavlink_lora_command_set_mode.h>
#include <mavlink_msgs/mavlink_lora_statustext.h>
#include <mavlink_msgs/mavlink_lora_heartbeat.h>
#include <mavlink_msgs/mavlink_lora_set_position_target_local_ned.h>
#include <mavlink_msgs/mavlink_lora_enable_offboard.h>
#include <mavlink_msgs/mavlink_lora_command_takeoff.h>
#include <mavlink_msgs/mavlink_lora_command_land.h>
#include <mavlink_msgs/mavlink_lora_command_reposition.h>
#include <mavlink_msgs/mavlink_lora_gps_raw.h>
#include <mavlink_msgs/mavlink_lora_radio_status.h>
#include <mavlink_msgs/mavlink_lora_command_message_interval.h>
#include <mavlink_msgs/mavlink_lora_battery_status.h>
#include <mavlink_msgs/mavlink_lora_command_long.h>

#include "mavlink_lora_lib.hpp"

/***************************************************************************/
/* defines */

#define DEFAULT_TIMEOUT_TIME 2 //1.5sec
#define MISSION_ITEM_TIMEOUT_TIME 2 //1.5sec
#define MISSION_ITEM_SINGLE_TIMEOUT_TIME 1.5 //2sec
#define MISSION_ITEM_SINGLE_ACK_TIMEOUT_TIME 20 //10sec TODO ideally it shouldn't time out as the final ack should always be ensured delivery with either accept or fail
#define MISSION_MAX_RETRIES 5
#define HEARTBEAT_RATE 0.5 //2 //hz
/***************************************************************************/
/* global variables */
ros::Time last_heard;
ros::Time last_heard_sys;
ros::Time status_msg_sent;
ros::Time heartbeat_msg_sent;
uint16_t sys_status_voltage_battery;
int8_t sys_status_battery_remaining;
uint16_t sys_status_cpu_load;
ros::Publisher msg_pub, pos_pub, status_pub, mission_ack_pub, command_ack_pub, statustext_pub, heartbeat_pub, mission_current_pub, gps_raw_pub, radio_status_pub, battery_status_pub, command_long_pub;
unsigned short msg_id_global_position_int_received;

/* Mission upload operations variables */
unsigned short mission_up_count = 0; /*< Total count of mission elements to be uploaded*/
unsigned short mission_partial_start_index, mission_partial_end_index; /*< vars used for append and partial upload */

int mission_up_index = -1; /*< Current mission item getting uploaded */
std::vector<mavlink_msgs::mavlink_lora_mission_item_int> missionlist; /*< list of all waypoints for upload */
bool mission_uploading = false; /*< Are we uploading a mission atm. Needed to know what messages/timeouts to react to*/
ros::Timer mission_up_timeout, mission_request_timeout;// = ros::NodeHandle::createTimer(ros::Duration(DEFAULT_TIMEOUT_TIME), mission_up_timeout_callback, true, false); /*< Timer for timeouts when doing mission transmissions */
int mission_retries = 0;
mavlink_msgs::mavlink_lora_mission_item_int last_single_mission_upload;

/* Command Protocol */
unsigned int confirmation = 0; // increments for each timeout of same command. Useful to monitor if a command should be killed
ros::Timer command_timeout; // timeout for commands.
mavlink_command_long_t last_cmd_long; // Saving parameters for resending last command

/* Lora Lib */
mavlink_lora_lib lora_lib;

/***************************************************************************/
/* Function Headers, for reference purposes */
void mission_up_count_timeout_callback(const ros::TimerEvent&);
void mission_up_item_timeout_callback(const ros::TimerEvent&);
void mission_request_timeout_callback(const ros::TimerEvent &);
void mission_clear_all_timeout_callback(const ros::TimerEvent&);
void command_long_timeout_callback(const ros::TimerEvent&);
void ml_send_mission_clear_all();
void ml_send_mission_count();
void ml_send_mission_item_int();
std::string mission_result_parser(uint8_t result);
std::string command_result_parser(uint8_t result);

/***************************************************************************/
void mavlink_tx_callback(const mavlink_msgs::mavlink_lora_msg::ConstPtr& msg)
{
	unsigned char *payload =	(unsigned char *) &msg->payload.front();
	lora_lib.ml_queue_msg_generic(msg->sys_id, msg->comp_id, msg->msg_id, msg->payload_len, payload);
}
/***************************************************************************/
void ml_send_status_msg(void)
{
	mavlink_msgs::mavlink_lora_status status;
	status.header.stamp = ros::Time::now();
	status.last_heard = last_heard;
	status.last_heard_sys_status = last_heard_sys;
	status.batt_volt = sys_status_voltage_battery;
    status.cpu_load = sys_status_cpu_load;
    status.batt_remaining = sys_status_battery_remaining;
	status.msg_sent_gcs = lora_lib.ml_messages_sent();
	status.msg_received_gcs = lora_lib.ml_messages_received();
	status.msg_dropped_gcs = lora_lib.ml_messages_crc_error();
	status.msg_lost_gcs = lora_lib.ml_messages_lost(1); // default sys id for PX4
	status_pub.publish(status);
	status_msg_sent = ros::Time::now();
}
/****************************** COMMAND_LONG *******************************/
void ml_send_command_long(unsigned short cmd_id, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    ROS_INFO_STREAM("Sending Command_long with id: " + std::to_string(cmd_id) + ", and confirmation: " + std::to_string(confirmation));

    //save command as last command
    last_cmd_long.command = cmd_id;
    last_cmd_long.param1 = p1;
    last_cmd_long.param2 = p2;
    last_cmd_long.param3 = p3;
    last_cmd_long.param4 = p4;
    last_cmd_long.param5 = p5;
    last_cmd_long.param6 = p6;
    last_cmd_long.param7 = p7;

    //queue msg
    lora_lib.ml_queue_msg_command_long(cmd_id, p1, p2, p3, p4, p5, p6, p7, confirmation);

    //start timeout
    ros::NodeHandle nh;
    command_timeout = nh.createTimer(ros::Duration(DEFAULT_TIMEOUT_TIME), command_long_timeout_callback, true, true);
}
/***************************************************************************/
void command_long_timeout_callback(const ros::TimerEvent&)
{
    //if timeout triggers, increment confirmation and resend last command
    confirmation++;

    ml_send_command_long(last_cmd_long.command, last_cmd_long.param1, last_cmd_long.param2, last_cmd_long.param3, last_cmd_long.param4, last_cmd_long.param5, last_cmd_long.param6, last_cmd_long.param7 );
}
/***************************************************************************/
std::string command_result_parser(uint8_t result)
{
    switch(result)
    {
        case 0:
            return "MAV_RESULT_ACCEPTED";
        case 1:
            return "MAV_RESULT_TEMPORARILY_REJECTED";
        case 2:
            return "MAV_RESULT_DENIED";
        case 3:
            return "MAV_RESULT_UNSUPPORTED";
        case 4:
            return "MAV_RESULT_FAILED";
        case 5:
            return "MAV_RESULT_IN_PROGRESS";
        default:
            return "DIDN'T RECOGNIZE RESULT CODE";
    }
}
/******************************* MISSIONS **********************************/
void ml_new_mission_callback(const mavlink_msgs::mavlink_lora_mission_list::ConstPtr& msg)
{
    //Received new mission on topic, start uploading
    mission_up_count = msg->waypoints.size();
    mission_up_index = -1;
    missionlist = msg->waypoints;

    ROS_INFO_STREAM("New mission. Length:" + std::to_string(mission_up_count));

    //Send mission count
    ml_send_mission_count();

    //Set status to uploading mission. Currently not used, but maybe use it to make sure you can't upload a new mission while another mission gets uploaded?
    mission_uploading = true;

}
/***************************************************************************/
void ml_mission_clear_all_callback(const std_msgs::Empty::ConstPtr& msg)
{
    //queue command for clearing all missions
    ml_send_mission_clear_all();

    // start timer
    ros::NodeHandle nh;
    mission_up_timeout = nh.createTimer(ros::Duration(DEFAULT_TIMEOUT_TIME), mission_clear_all_timeout_callback, true, true);
}
/***************************************************************************/
void ml_send_mission_count()
{
    ROS_INFO_STREAM("Sending mission count");
    // queue msg
    lora_lib.ml_queue_msg_mission_count(mission_up_count);

    // start timer
    ros::NodeHandle nh;
    mission_up_timeout = nh.createTimer(ros::Duration(DEFAULT_TIMEOUT_TIME), mission_up_count_timeout_callback, true, true);
}
/***************************************************************************/
void ml_send_mission_clear_all()
{
    ROS_INFO_STREAM("Sending mission clear all");
    // queue msg
    lora_lib.ml_queue_msg_mission_clear_all();

    // start timer
    ros::NodeHandle nh;
    mission_up_timeout = nh.createTimer(ros::Duration(DEFAULT_TIMEOUT_TIME), mission_clear_all_timeout_callback, true, true);
}
/***************************************************************************/
void ml_send_mission_item_int()
{
    //send mission item.
    mavlink_msgs::mavlink_lora_mission_item_int item = missionlist[mission_up_index];
    lora_lib.ml_queue_msg_mission_item_int(item.param1, item.param2, item.param3, item.param4, item.x, item.y, item.z, item.seq, item.command, item.frame, item.current, item.autocontinue);

    //start timeout
    ros::NodeHandle nh;
    mission_up_timeout = nh.createTimer(ros::Duration(MISSION_ITEM_TIMEOUT_TIME), mission_up_item_timeout_callback, true, true);

    ROS_INFO_STREAM("Sent mission item seq: " + std::to_string(mission_up_index));
}
/***************************************************************************/
void mission_up_item_timeout_callback(const ros::TimerEvent&)
{
    //increment retries
    mission_retries++;

    //check if retries has been exceeded
    if (mission_retries > MISSION_MAX_RETRIES)
    {
        //publish error on ack topic
        mavlink_msgs::mavlink_lora_mission_ack msg;
        msg.result = 20;
        msg.result_text = mission_result_parser(20);
        mission_ack_pub.publish(msg);

        //cancel command
        mission_up_index = -1;
        mission_up_count = 0;
        missionlist.clear();
        mission_uploading = false;

        //reset retries
        mission_retries = 0;

        //debug
        ROS_INFO_STREAM("MAX RETRIES REACHED");

        return;
    }

    //if timeout triggers, resend last mission item
    ml_send_mission_item_int();
}
/***************************************************************************/
void mission_clear_all_timeout_callback(const ros::TimerEvent&)
{
    //increment retries
    mission_retries++;

    //check if retries has been exceeded
    if (mission_retries > MISSION_MAX_RETRIES)
    {
        //publish error on ack topic
        mavlink_msgs::mavlink_lora_mission_ack msg;
        msg.result = 20;
        msg.result_text = mission_result_parser(20);
        mission_ack_pub.publish(msg);

        //reset retries
        mission_retries = 0;

        //debug
        ROS_INFO_STREAM("MAX RETRIES REACHED");

        return;
    }

    //if timeout triggers, resend
    ml_send_mission_clear_all();
}
/***************************************************************************/
void mission_up_count_timeout_callback(const ros::TimerEvent&)
{
    //increment retries
    mission_retries++;

    //check if retries has been exceeded
    if (mission_retries > MISSION_MAX_RETRIES)
    {
        //publish error on ack topic
        mavlink_msgs::mavlink_lora_mission_ack msg;
        msg.result = 20;
        msg.result_text = mission_result_parser(20);
        mission_ack_pub.publish(msg);

        //cancel command
        mission_uploading = false;

        //reset retries
        mission_retries = 0;

        //debug
        ROS_INFO_STREAM("MAX RETRIES REACHED");

        return;
    }

    //if timeout triggers, resend count message
    ml_send_mission_count();
}
/***************************************************************************/
void mission_request_timeout_callback(const ros::TimerEvent &)
{
    //publish error on ack topic
    mavlink_msgs::mavlink_lora_mission_ack msg;
    msg.result = 21;
    msg.result_text = mission_result_parser(21); //ack timeout error
    mission_ack_pub.publish(msg);
    ROS_WARN_STREAM("Mission request timed out");
}
/***************************************************************************/
std::string mission_result_parser(uint8_t result)
{
    switch(result)
    {
        case 0:
            return "MAV_MISSION_ACCEPTED";
        case 1:
            return "MAV_MISSION_ERROR";
        case 2:
            return "MAV_MISSION_UNSUPPORTED_FRAME";
        case 3:
            return "MAV_MISSION_UNSUPPORTED";
        case 4:
            return "MAV_MISSION_NO_SPACE";
        case 5:
            return "MAV_MISSION_INVALID";
        case 6:
            return "MAV_MISSION_INVALID_PARAM1";
        case 7:
            return "MAV_MISSION_INVALID_PARAM2";
        case 8:
            return "MAV_MISSION_INVALID_PARAM3";
        case 9:
            return "MAV_MISSION_INVALID_PARAM4";
        case 10:
            return "MAV_MISSION_INVALID_PARAM5_X";
        case 11:
            return "MAV_MISSION_INVALID_PARAM6_Y";
        case 12:
            return "MAV_MISSION_INVALID_PARAM7";
        case 13:
            return "MAV_MISSION_INVALID_SEQUENCE";
        case 14:
            return "MAV_MISSION_DENIED";
        case 20:
            return "MAV_MISSION_MAX_RETRIES"; //Custom error indicating aborting due to max retries with no success
        case 21:
            return "MAV_MISSION_ACK_TIMEOUT"; //Timeout on ack for mission
        default:
            return "DIDN'T RECOGNIZE RESULT CODE";
    }
}
/************************* INTERFACE COMMANDS ******************************/
void ml_command_arm_disarm_callback(const std_msgs::Bool::ConstPtr& msg)
{
    //arm if true, disarm if false
    //send msg
    ml_send_command_long(MAVLINK_MSG_ID_COMPONENT_ARM_DISARM, msg->data, 0, 0, 0, 0, 0, 0);
}
void ml_command_start_mission_callback(const mavlink_msgs::mavlink_lora_command_start_mission::ConstPtr& msg)
{
    //start mission, first and last item as parameter
    //send msg
    ml_send_command_long(MAVLINK_MSG_ID_MISSION_START, msg->first_item, msg->last_item, 0, 0, 0, 0, 0);
}
void ml_command_set_mode_callback(const mavlink_msgs::mavlink_lora_command_set_mode::ConstPtr& msg)
{
    ml_send_command_long(MAVLINK_MSG_ID_DO_SET_MODE, msg->mode, msg->custom_mode, msg->custom_sub_mode, 0, 0, 0, 0);
}
void ml_command_takeoff_callback(const mavlink_msgs::mavlink_lora_command_takeoff::ConstPtr& msg)
{
    ml_send_command_long(MAVLINK_MSG_ID_NAV_TAKEOFF, msg->pitch, 0, 0, msg->yaw_angle, msg->lat, msg->lon, msg->alt); //unchanged yaw_angle == nanf()
}
void ml_command_land_callback(const mavlink_msgs::mavlink_lora_command_land::ConstPtr& msg)
{
    ml_send_command_long(MAVLINK_MSG_ID_NAV_LAND, msg->abort_alt, msg->precision_land_mode, 0, msg->yaw_angle, msg->lat, msg->lon, msg->altitude);
}
void ml_command_do_reposition_callback(const mavlink_msgs::mavlink_lora_command_reposition::ConstPtr& msg)
{
    //
    ml_send_command_long(MAVLINK_MSG_ID_SET_REPOSITION, msg->ground_speed, 0, 0, msg->yaw_heading, msg->lat, msg->lon, msg->alt);
}
void ml_command_do_pause_continue_callback(const std_msgs::Bool::ConstPtr& msg)
{
    // 1 == continue mission, 0 == hold position
    ml_send_command_long(MAVLINK_MSG_ID_DO_PAUSE_CONTINUE, msg->data, 0, 0, 0, 0, 0, 0);
}
void ml_command_message_interval(const mavlink_msgs::mavlink_lora_command_message_interval::ConstPtr& msg)
{
    //set message interval
    ml_send_command_long(MAVLINK_MSG_ID_SET_MESSAGE_INTERVAL, msg->message_id, msg->interval, 0, 0, 0, 0, 0);
}
/***************************** CALIBRATION ***********************************/
void ml_command_preflight_calibration_compass_callback(const std_msgs::Empty)
{
    ml_send_command_long(MAVLINK_MSG_ID_PREFLIGHT_CALIBRATION, 0, 1, 0, 0, 0, 0, 0);
}
/***************************** HEARTBEAT ***********************************/
void ml_send_heartbeat_callback(const mavlink_msgs::mavlink_lora_heartbeat::ConstPtr& msg)
{
    //queue heartbeat
    lora_lib.ml_queue_msg_heartbeat(msg->type, msg->autopilot, msg->base_mode, msg->custom_mode, msg->system_status, msg->system_id);
}
void ml_send_heartbeat()
{
    //queue heartbeat
    lora_lib.ml_queue_msg_heartbeat(6, 8, 136, 0, 4, 255);

    heartbeat_msg_sent = ros::Time::now();
}

/************** PARSE From FC *************/
void parse_msg_from_fc(unsigned char *msg)
{
    last_heard = ros::Time::now();

    // extract message info
    struct mavlink_msg_t m;
    m.payload_len = msg[ML_POS_PAYLOAD_LEN];
    m.seq = msg[ML_POS_PACKET_SEQ];
    m.sys_id = msg[ML_POS_SYS_ID];
    m.comp_id = msg[ML_POS_COMP_ID];
    m.msg_id = msg[ML_POS_MSG_ID];

    for (auto i=0; i<m.payload_len; i++)
        m.payload.push_back(msg[ML_POS_PAYLOAD + i]);

    unsigned char crc_lsb = msg[6 + m.payload_len];
    unsigned char crc_msb = msg[7 + m.payload_len];
    m.checksum = (8 << crc_msb) | crc_lsb;

    //Publish raw message on mavlink_rx
    mavlink_msgs::mavlink_lora_msg raw_msg;
    raw_msg.header.stamp = last_heard;
    raw_msg.seq = m.seq;
    raw_msg.sys_id = m.sys_id;
    raw_msg.comp_id = m.comp_id;
    raw_msg.msg_id = m.msg_id;
    raw_msg.payload = m.payload;
    raw_msg.payload_len = m.payload_len;
    msg_pub.publish(raw_msg);

    // TODO Moving filtering to mission_computers side. Should limit the px4 messages directly on the PX4 with message_interval
//    int filter_msg[] = {2, 30, 31, 32, 35, 36, 65, 69, 70, 74, 83, 105, 140, 141, 147, 230, 241,  245, 40, 51};
//
//    for (auto id : filter_msg)
//        if (m.msg_id == id)
//            send_gcs = false;

    // handle state messages
    if	(m.msg_id == MAVLINK_MSG_ID_SYS_STATUS)
    {
        last_heard_sys = last_heard;
        mavlink_sys_status_t sys_status = mavlink_lora_lib::ml_unpack_msg_sys_status (&m.payload.front());
        sys_status_voltage_battery = sys_status.voltage_battery;
        sys_status_battery_remaining = sys_status.battery_remaining;
        sys_status_cpu_load = sys_status.load;
    }

    //handle statustext
    if (m.msg_id == MAVLINK_MSG_ID_STATUSTEXT)
    {
        //unpack msg
        mavlink_statustext_t stat_text = mavlink_lora_lib::ml_unpack_msg_statustext(&m.payload.front());

        //create ros msg
        mavlink_msgs::mavlink_lora_statustext status_text;
        status_text.severity = stat_text.severity;
        status_text.text = stat_text.text;

        //publish
        statustext_pub.publish(status_text);
    }

    // handle pos messages
    if (m.msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
    {
        mavlink_msgs::mavlink_lora_pos pos;
        pos.header.stamp = last_heard;
        mavlink_global_position_int_t glob_pos = mavlink_lora_lib::ml_unpack_msg_global_position_int (&m.payload.front());
        pos.time_usec = (uint64_t) glob_pos.time_boot_ms*1000;
        pos.lat = glob_pos.lat / 1e7;
        pos.lon = glob_pos.lon / 1e7;
        pos.alt = glob_pos.alt / 1e3;
        pos.relative_alt = glob_pos.relative_alt / 1e3;
        pos.heading = glob_pos.hdg /100;
        pos_pub.publish(pos);
        msg_id_global_position_int_received = true;
    }

    //Send raw until system starts to report estimated position with global_pos
    if (m.msg_id == MAVLINK_MSG_ID_GPS_RAW_INT && msg_id_global_position_int_received == false)
    {
        mavlink_msgs::mavlink_lora_pos pos;
        pos.header.stamp = last_heard;
        mavlink_gps_raw_int_t gri = mavlink_lora_lib::ml_unpack_msg_gps_raw_int (&m.payload.front());
        pos.time_usec = gri.time_usec;
        pos.lat = gri.lat / 1e7;
        pos.lon = gri.lon / 1e7;
        pos.alt = gri.alt / 1e3;
        pos.relative_alt = -1;
        pos.heading = -1;
        pos_pub.publish(pos);
    }

    //always output gps_raw as seperate message as it gives gps_fix type, groundspeed, hdop, and vdop
    if (m.msg_id == MAVLINK_MSG_ID_GPS_RAW_INT)
    {
        mavlink_msgs::mavlink_lora_gps_raw raw;
        mavlink_gps_raw_int_t gri = mavlink_lora_lib::ml_unpack_msg_gps_raw_int(&m.payload.front());
        raw.time_usec = gri.time_usec;
        raw.fix_type = gri.fix_type;
        raw.lat = gri.lat / 1e7;
        raw.lon = gri.lon / 1e7;
        raw.alt = gri.alt / 1e3; //mm -> m
        raw.eph = gri.eph;
        raw.epv = gri.epv;
        raw.vel = gri.vel / 100; //cm/s -> m/s
        raw.cog = gri.cog / 100; //cdeg -> deg
        raw.satellites_visible = gri.satellites_visible;

        gps_raw_pub.publish(raw);
    }

    //handle mission ack
    if (m.msg_id == MAVLINK_MSG_ID_MISSION_ACK)
    {
        //stop timer
        mission_up_timeout.stop();

        //unpack
        mavlink_mission_ack_t ack = mavlink_lora_lib::ml_unpack_msg_mission_ack(&m.payload.front());

        //reset retries
        mission_retries = 0;

        //stop upload flag
        mission_uploading = false;

        //respond back with result
        mavlink_msgs::mavlink_lora_mission_ack msg;
        msg.result = ack.type;
        msg.result_text = mission_result_parser(ack.type);

        mission_ack_pub.publish(msg);

        // DEBUG
        ROS_DEBUG_STREAM("Mission ack from FC: " + msg.result_text);
    }

    //handle command ack
    if (m.msg_id == MAVLINK_MSG_ID_COMMAND_ACK)
    {
        //stop timer
        command_timeout.stop();

        //reset confirmation
        confirmation = 0;

        //unpack
        mavlink_command_ack_t ack = mavlink_lora_lib::ml_unpack_msg_command_ack(&m.payload.front());

        //respond back with result
        mavlink_msgs::mavlink_lora_command_ack ack_msg;
        ack_msg.command = ack.command;
        ack_msg.result = ack.result;
        ack_msg.result_text = command_result_parser(ack.result);

        //publish
        command_ack_pub.publish(ack_msg);

        //DEBUG
        ROS_INFO_STREAM(ack_msg.result_text);
    }

    //handle current mission
    if (m.msg_id == MAVLINK_MSG_ID_MISSION_CURRENT)
    {
        // make msg to MQTT format and send
        uint16_t seq = mavlink_lora_lib::ml_unpack_msg_mission_current(&m.payload.front());

        std_msgs::UInt16 rosmsg;
        rosmsg.data = seq;

        //publish
        mission_current_pub.publish(rosmsg);
    }

    // handle heartbeat messages
    if (m.msg_id == MAVLINK_MSG_ID_HEARTBEAT)
    {
        mavlink_heartbeat_t hb = mavlink_lora_lib::ml_unpack_msg_heartbeat(&m.payload.front());

        //msg
        mavlink_msgs::mavlink_lora_heartbeat hb_msg;

        hb_msg.autopilot = hb.autopilot;
        hb_msg.base_mode = hb.base_mode;
        hb_msg.custom_mode = hb.custom_mode;
        hb_msg.type = hb.type;
        hb_msg.system_id = hb.system_id;
        hb_msg.system_status = hb.system_status;

        //publish
        heartbeat_pub.publish(hb_msg);
    }

    //Handle Mission upload messages
    /* Forcing it to use INT variants for best possible precision. */
    if (m.msg_id == MAVLINK_MSG_ID_MISSION_REQUEST_INT || m.msg_id == MAVLINK_MSG_ID_MISSION_REQUEST)
    {
        //do not act on this unless we are uploading missions... Otherwise we can throw expection as our saved
        //mission items are empty
        if (!mission_uploading)
            return;

        // stop mission ack timeout
//        mission_request_timeout.stop();
//        ros::NodeHandle nh;
//        mission_request_timeout = nh.createTimer(ros::Duration(MISSION_ITEM_TIMEOUT_TIME), mission_request_timeout_callback, true, true);

        //reset retries
        mission_retries = 0;

        //unpack and update requested seq
        mavlink_mission_request_int_t request = mavlink_lora_lib::ml_unpack_msg_mission_request_int(&m.payload.front());

        // Possible multipath causes to receive multiple requests
        if(mission_up_index == request.seq)
            ROS_INFO_STREAM("Already sent this sequence number once");
        else
        {
            //stop timer
            mission_up_timeout.stop();

            mission_up_index = request.seq;

            //send next item
            ml_send_mission_item_int();

            ROS_INFO_STREAM("Next item asked for:" + std::to_string(mission_up_index)); //TODO agree on what to print, and if debug prints can be enabled from launchfile or so?
        }
    }

    //handle radio_status
    if (m.msg_id == MAVLINK_MSG_ID_RADIO_STATUS)
    {
        // unpack msg
        mavlink_radio_status_t status = mavlink_lora_lib::ml_unpack_msg_radio_status(&m.payload.front());
        mavlink_msgs::mavlink_lora_radio_status radioStatus;

        int rssi, remrssi, rssi_limited, remrssi_limited;

        // 3DR Si1k radio needs rssi to be converted to db. This is the same way QGroundControl calculates it
        if (m.sys_id == '3' && m.comp_id == 'D') {
            /* Per the Si1K datasheet figure 23.25 and SI AN474 code
             * samples the relationship between the RSSI register
             * and received power is as follows:
             *
             *                       10
             * inputPower = rssi * ------ 127
             *                       19
             *
             * Additionally limit to the only realistic range [-120,0] dBm
             * signal_dbm = (RSSI / 1.9) - 127.
             */

            rssi = static_cast<int>(std::round(static_cast<double>(status.rssi) / 1.9 - 127.0));
            rssi_limited = std::min(std::max(rssi, -120), 0);

            remrssi = static_cast<int>(std::round(static_cast<double>(status.remrssi) / 1.9 - 127.0));
            remrssi_limited = std::min(std::max(remrssi, -120), 0);
        } else {
            rssi_limited = (int8_t) status.rssi;
            remrssi_limited = (int8_t) status.remrssi;
        }

        // build ros package
        radioStatus.rssi = rssi_limited;
        radioStatus.remrssi = remrssi_limited;
        radioStatus.noise = status.noise;
        radioStatus.remnoise = status.remnoise;
        radioStatus.txbuf = status.txbuf;
        radioStatus.rxerrors = status.rxerrors;
        radioStatus.fixed = status.fixed;

        // publish
        radio_status_pub.publish(radioStatus);
    }

    //handle battery_status
    if (m.msg_id == MAVLINK_MSG_ID_BATTERY_STATUS)
    {
        // make msg to ROS format and send
        mavlink_battery_status_t bs = mavlink_lora_lib::ml_unpack_msg_battery_status(&m.payload.front());

        mavlink_msgs::mavlink_lora_battery_status ros_msg;
        ros_msg.id = bs.id;
        ros_msg.type = bs.type;
        ros_msg.battery_remaining = bs.battery_remaining;
        ros_msg.battery_function = bs.battery_function;
        ros_msg.current_battery = bs.current_battery;
        ros_msg.temperature = bs.temperature;
        ros_msg.energy_consumed = bs.energy_consumed;
        ros_msg.current_consumed = bs.current_consumed;
        ros_msg.voltages = std::vector<unsigned short> (std::begin(bs.voltages), std::end(bs.voltages));

        // publish
        battery_status_pub.publish(ros_msg);

        // update total voltage for system id
        sys_status_battery_remaining = bs.battery_remaining;

        // get combined voltage from summing individual cells. If cell is UINT16_MAX battery doesn't include that cell
        float sum_voltage = 0.f;
        for (auto c : bs.voltages) {
            if (c != numeric_limits<unsigned short>::max())
                sum_voltage += c;
        }

        sys_status_voltage_battery = sum_voltage;
    }

    //handle command long
    if (m.msg_id == MAVLINK_MSG_ID_COMMAND_LONG) {
        //unpack msg
        mavlink_command_long_t current_command = mavlink_lora_lib::ml_unpack_msg_command_long(&m.payload.front());

        mavlink_msgs::mavlink_lora_command_long cmd_long_msg;

        cmd_long_msg.command = current_command.command;
        cmd_long_msg.param1 = current_command.param1;
        cmd_long_msg.param2 = current_command.param2;
        cmd_long_msg.param3 = current_command.param3;
        cmd_long_msg.param4 = current_command.param4;
        cmd_long_msg.param5 = current_command.param5;
        cmd_long_msg.param6 = current_command.param6;
        cmd_long_msg.param7 = current_command.param7;
        cmd_long_msg.target_component = current_command.target_component;
        cmd_long_msg.target_system = current_command.target_system;
        cmd_long_msg.confirmation = current_command.confirmation;

        command_long_pub.publish(cmd_long_msg);
    }
}

/******************************* MAIN **************************************/
int main (int argc, char** argv)
{	ros::init(argc, argv, "hd_mavlink_router_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~"); // private parameters

    /* read from parameter server */
    std::string serial_device;
    int serial_baudrate;
    bool heartbeats_mock;
    nh.param<std::string> ("serial_device", serial_device, "/dev/ttyUSB0");
    nh.param<int> ("serial_baudrate", serial_baudrate, 57600);
    nh.param<bool> ("heartbeats", heartbeats_mock, false);

    /* initialize variables */
    ros::Time begin = ros::Time::now();
    msg_id_global_position_int_received = false;

    /* initialize serial port */
    ROS_INFO_STREAM("Initiating mavlink_router and mavlink library");
    ROS_INFO("Opening serial device: %s baudrate %d", serial_device.c_str(), serial_baudrate);
    if (lora_lib.init("FC", serial_device, serial_baudrate, parse_msg_from_fc) == -1)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }
    ROS_INFO_STREAM("Serial Port initialized");

    ROS_DEBUG_STREAM("Setting up ROS topics");
    ros::Subscriber write_sub = n.subscribe("mavlink_tx", 10, mavlink_tx_callback);
    msg_pub = n.advertise<mavlink_msgs::mavlink_lora_msg>("mavlink_rx", 1);
    pos_pub = n.advertise<mavlink_msgs::mavlink_lora_pos>("mavlink_pos", 1);
    status_pub = n.advertise<mavlink_msgs::mavlink_lora_status>("mavlink_status", 1);
    statustext_pub = n.advertise<mavlink_msgs::mavlink_lora_statustext>("mavlink_statustext", 1);
    gps_raw_pub = n.advertise<mavlink_msgs::mavlink_lora_gps_raw>("mavlink_gps_raw", 1);
    radio_status_pub = n.advertise<mavlink_msgs::mavlink_lora_radio_status>("mavlink_radio_status", 1);
    battery_status_pub = n.advertise<mavlink_msgs::mavlink_lora_battery_status>("mavlink_battery_status", 1);
    command_long_pub = n.advertise<mavlink_msgs::mavlink_lora_command_long>("mavlink_interface/command/command_long", 1);

    /* Interface subscribers */
    ros::Subscriber mission_upload_sub = n.subscribe("mavlink_interface/mission/mavlink_upload_mission", 1, ml_new_mission_callback);
    ros::Subscriber mission_clear_all_sub = n.subscribe("mavlink_interface/mission/mavlink_clear_all", 1, ml_mission_clear_all_callback);
    ros::Subscriber command_arm_disarm_sub = n.subscribe("mavlink_interface/command/arm_disarm", 1, ml_command_arm_disarm_callback);
    ros::Subscriber command_start_mission_sub = n.subscribe("mavlink_interface/command/start_mission", 1, ml_command_start_mission_callback);
    ros::Subscriber command_set_mode_sub = n.subscribe("mavlink_interface/command/set_mode", 1, ml_command_set_mode_callback);
    ros::Subscriber command_takeoff_sub = n.subscribe("mavlink_interface/command/takeoff", 1, ml_command_takeoff_callback);
    ros::Subscriber command_land_sub = n.subscribe("mavlink_interface/command/land", 1, ml_command_land_callback);
    ros::Subscriber command_reposition_sub = n.subscribe("mavlink_interface/command/reposition", 1, ml_command_do_reposition_callback);
    ros::Subscriber command_pause_continue_sub = n.subscribe("mavlink_interface/command/pause_continue", 1, ml_command_do_pause_continue_callback);
    ros::Subscriber command_message_interval_sub = n.subscribe("mavlink_interface/command/message_interval", 1, ml_command_message_interval);

    /* Interface publishers */
    mission_ack_pub = n.advertise<mavlink_msgs::mavlink_lora_mission_ack>("mavlink_interface/mission/ack", 1);
    mission_current_pub = n.advertise<std_msgs::UInt16>("mavlink_interface/mission/current", 1);
    command_ack_pub = n.advertise<mavlink_msgs::mavlink_lora_command_ack>("mavlink_interface/command/ack", 1);

    /* Heartbeat */
    ros::Subscriber heartbeat_sub = n.subscribe("mavlink_heartbeat_tx", 1, ml_send_heartbeat_callback);
    heartbeat_pub = n.advertise<mavlink_msgs::mavlink_lora_heartbeat>("mavlink_heartbeat_rx", 1);

    ROS_DEBUG_STREAM("Starting main loop");
    /* ros main loop */
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();

        /* check if it is time to send a status message */
        if (status_msg_sent + ros::Duration(1) <= ros::Time::now())
            ml_send_status_msg();

        if (heartbeat_msg_sent + ros::Duration(HEARTBEAT_RATE) <= ros::Time::now() && heartbeats_mock)
            ml_send_heartbeat();

        //update tick for mavlink_lib
        lora_lib.update();

        //sleep
        loop_rate.sleep();
    }

    /* closing down */
    ROS_INFO("Shutting down router and mavlink library");
    lora_lib.shutdown();
}
/***************************************************************************/
