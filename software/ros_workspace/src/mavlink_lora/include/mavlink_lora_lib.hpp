//
// Created by Crow on 25-03-2019.
//

#ifndef MAVLINK_BRIDGE_MAVLINK_LORA_LIB_HPP
#define MAVLINK_BRIDGE_MAVLINK_LORA_LIB_HPP

///Includes
#include <string>
#include <vector>
#include <functional>
#include "checksum.h"
#include "mavlink_messages.hpp"
#include <iostream>
#include <sys/time.h>
#include <cstring>
#include <unordered_map>
#include <cmath>

extern "C"
{
#include "../include/serial.h"
}

using namespace std;

/* Defines */
#define RX_BUF_SIZE	16000 /* size of the receive buffer, depends on system and application */
#define PARAM_TOUT 2000 /* [ms] timeout for other msgs when exchanging parameters */
#define MISSION_TOUT 2000 /* [ms] timeout for other msgs when exchanging mission */
#define TX_BUF_SIZE 200

struct LostMessages
{
    unsigned long last_seq = 0;
    unsigned long lostMessages = 0;
};

class mavlink_lora_lib {

public:
    mavlink_lora_lib();

    /// program workers
    int init(string identify_name, string serial_device, int baudrate, function<void(unsigned char*)> callback);
    void update();
    void shutdown();

    /// Serial
    void ml_forward_msg(unsigned char * inc_buf);

    /// Static unpack functions
    static mavlink_sys_status_t ml_unpack_msg_sys_status (unsigned char *payload);
    static mavlink_attitude_t ml_unpack_msg_attitude (unsigned char *payload);
    static mavlink_gps_raw_int_t ml_unpack_msg_gps_raw_int (unsigned char *payload);
    static mavlink_global_position_int_t ml_unpack_msg_global_position_int (unsigned char *payload);
    static mavlink_mission_count_t ml_unpack_msg_mission_count (unsigned char *payload);
    static mavlink_mission_partial_write_list_t ml_unpack_msg_mission_partial_write_list (unsigned char *payload);
    static mavlink_mission_item_t ml_unpack_msg_mission_item (unsigned char *payload);
    static mavlink_mission_item_int_t ml_unpack_msg_mission_item_int (unsigned char *payload);
    static unsigned short ml_unpack_msg_mission_current (unsigned char *payload);
    static mavlink_mission_request_int_t ml_unpack_msg_mission_request_int (unsigned char *payload);
    static mavlink_mission_ack_t ml_unpack_msg_mission_ack (unsigned char *payload);
    static mavlink_statustext_t ml_unpack_msg_statustext (unsigned char *payload);
    static mavlink_command_ack_t ml_unpack_msg_command_ack (unsigned char *payload);
    static mavlink_heartbeat_t ml_unpack_msg_heartbeat (unsigned char *payload);
    static mavlink_radio_status_t ml_unpack_msg_radio_status (unsigned char *payload);
    static mavlink_system_time_t ml_unpack_msg_system_time (unsigned char *payload);
    static mavlink_altitude_t ml_unpack_msg_altitude (unsigned char *payload);
    static mavlink_battery_status_t ml_unpack_msg_battery_status (unsigned char *payload);
    static mavlink_vibration_t ml_unpack_msg_vibration (unsigned char *payload);
    static mavlink_extended_sys_status_t ml_unpack_msg_extended_sys_status (unsigned char *payload);
    static mavlink_command_long_t ml_unpack_msg_command_long (unsigned char *payload);
    static mavlink_home_position_t ml_unpack_msg_home_position (unsigned char *payload);

    /// Functions
    void ml_queue_msg_generic (unsigned char sys_id, unsigned char comp_id, unsigned char msg_id, unsigned char payload_len, unsigned char *payload);
    void ml_queue_msg_mission_request (unsigned short seq);
    void ml_queue_msg_mission_request_list ();
    void ml_queue_msg_mission_ack (uint8_t result);
    void ml_queue_msg_mission_count (unsigned short count);
    void ml_queue_msg_mission_clear_all ();
    void ml_queue_msg_mission_item_int (float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z, unsigned short seq, unsigned short command, unsigned char frame, unsigned char current, unsigned char autocontinue);
    void ml_queue_msg_command_long (unsigned short cmd_id, float param1, float param2, float param3, float param4, float param5, float param6, float param7, unsigned int confirmation);
    void ml_queue_msg_command_ack (unsigned short cmd_id, unsigned char result);
    void ml_queue_msg_heartbeat(unsigned char type, unsigned char autopilot, unsigned char base_mode, unsigned long custom_mode, unsigned char system_status, unsigned char system_id);

    unsigned long ml_messages_sent();
    unsigned long ml_messages_received();
    unsigned long ml_messages_crc_error();
    unsigned long ml_messages_lost(unsigned char sys_id);

private:
    // serial device
    int ser_ref;
    struct termios oldtio;

    string identifier_name;

    // Buffers
    unsigned char txbuf[TX_BUF_SIZE];
    unsigned char rxbuf_serial_read[RX_BUF_SIZE];   //Required as raw buffer to store inc. bytes from serial.
    unsigned char rxbuf[RX_BUF_SIZE];               //Is the internal queue of bytes as we receive them.

    short txbuf_cnt = 0;
    short rxbuf_cnt = 0;
    char tx_seq;
    short msg_begin = -1;

    unsigned long msg_rx_cnt, msg_crc_err, msg_tx_cnt;
    unsigned long msg_buf_overflow;

    // hashmap for last seq received and number of lost messages
    unordered_map<unsigned char, LostMessages> lost_messages_map;

    unsigned char recorded_sysid;


    unsigned char do_send_msg;
    unsigned long param_mission_tout;

    unsigned long secs_init;

    bool debug = false;

    unsigned long millis();

    //CRC Check
    const vector<unsigned char> MAVLINK_MESSAGE_CRCS_V1 = {50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119, 191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 134, 219, 208, 188, 84, 22, 19, 21, 134, 0, 78, 68, 189, 127, 154, 21, 21, 144, 1, 234, 73, 181, 22, 83, 167, 138, 234, 240, 47, 189, 52, 174, 229, 85, 159, 186, 72, 0, 0, 0, 0, 92, 36, 71, 98, 0, 0, 0, 0, 0, 134, 205, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 69, 101, 50, 202, 17, 162, 0, 0, 0, 0, 0, 0, 207, 0, 0, 0, 163, 105, 151, 35, 150, 0, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0};

    /// Serial Functions
    short ml_rx_update(unsigned long ms, unsigned char *buf, short buf_cnt);
    void ml_tx_update();
    short ml_queue_msg(unsigned char *buf, unsigned char sys_id);

    /// Callback function
    function<void(unsigned char *msg)> parse_msg_callback;
};


#endif //MAVLINK_BRIDGE_MAVLINK_LORA_LIB_HPP
