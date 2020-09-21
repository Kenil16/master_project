#include <utility>

//
// Created by Crow on 25-03-2019.
//

#include "../include/mavlink_lora_lib.hpp"

mavlink_lora_lib::mavlink_lora_lib() {
    /* initialize variables */
    msg_rx_cnt = 0;
    msg_crc_err = 0;
    msg_tx_cnt = 0;
    msg_buf_overflow = 0;
    tx_seq = 1;
    recorded_sysid = 0;
    param_mission_tout = 0;
}
int mavlink_lora_lib::init(string identify_name, string serial_device, int baudrate, function<void(unsigned char*)> callback)
{
    //save name
    identifier_name = std::move(identify_name);

    //save callback
    parse_msg_callback = callback;

    printf("Opening serial device for %s: %s baudrate %d \n", identifier_name.c_str(), serial_device.c_str(), baudrate);
    if (ser_open(&ser_ref, &oldtio, (char *) serial_device.c_str(), baudrate)) {
        cout << "port couldn't be opened" << endl;
        return -1;
    }
}
void mavlink_lora_lib::update()
{
    int cnt = ser_receive(ser_ref, rxbuf_serial_read, RX_BUF_SIZE);
    if(cnt > 0)
    {
        ml_rx_update(millis(), rxbuf_serial_read, cnt);
    }
}
void mavlink_lora_lib::shutdown()
{
    //close serial ports
    ser_close(ser_ref, oldtio);

    cout << "Closed serial ports for lib with name: " << identifier_name <<  endl;
}
/***************************************************************************/
short mavlink_lora_lib::ml_queue_msg(unsigned char *buf, unsigned char sys_id)
{
    unsigned char msg_id;
    unsigned char payload_len;
    unsigned short crc;
    unsigned char crc_extra;

    msg_tx_cnt++;

    /* encode the generic part of the header */
    buf[ML_POS_IDENT] = ML_NEW_PACKET_IDENT_V10;

    if (tx_seq == 0)
        tx_seq = 1;

    /* add checksum */
    msg_id = buf[ML_POS_MSG_ID];
    payload_len = buf[ML_POS_PAYLOAD_LEN];
    crc = crc_calculate(buf+1, payload_len+5);
    crc_extra = MAVLINK_MESSAGE_CRCS_V1[msg_id];
    crc_accumulate(crc_extra, &crc);
    (buf+payload_len+ 6)[0] = (crc & 0xff);
    (buf+payload_len+ 7)[0] = (crc >> 8);

    /*printf ("%d\n", crc);
    int i;
    for (i=0; i<buf[ML_POS_PAYLOAD_LEN]+8; i++)
    {
            printf ("%03d ", buf[i]);
    }
    printf ("\n"); */

    /* update txbuf length */
    txbuf_cnt += buf[ML_POS_PAYLOAD_LEN] + 8;

    // update tx aka. send the bytes TODO might give issues?
    ml_tx_update();
}
void mavlink_lora_lib::ml_queue_msg_generic (unsigned char sys_id, unsigned char comp_id, unsigned char msg_id, unsigned char payload_len, unsigned char *payload)
{
    unsigned char i, len;
    unsigned char *buf = (txbuf + txbuf_cnt);

    /* encode part of the header */
    buf[ML_POS_PAYLOAD_LEN] = payload_len;
    buf[ML_POS_SYS_ID] = sys_id;
    buf[ML_POS_COMP_ID] = comp_id;
    buf[ML_POS_MSG_ID] = msg_id;

    /* param_index */
    for (i=0; i<payload_len; i++)
        buf[ML_POS_PAYLOAD + i] = payload[i];

    /* queue message */
    recorded_sysid = 0;
    ml_queue_msg(buf, sys_id);
}
void mavlink_lora_lib::ml_forward_msg(unsigned char *inc_buf)
{
    unsigned char *buf = (txbuf + txbuf_cnt);

    uint8_t payload_len = inc_buf[ML_POS_PAYLOAD_LEN];
    uint8_t sys_id = inc_buf[ML_POS_SYS_ID];

    buf[ML_POS_PAYLOAD_LEN] = payload_len;
    buf[ML_POS_SYS_ID] = sys_id;
    buf[ML_POS_COMP_ID] = inc_buf[ML_POS_COMP_ID];
    buf[ML_POS_MSG_ID] = inc_buf[ML_POS_MSG_ID];

    for (auto i=0; i<payload_len; i++) {
        buf[ML_POS_PAYLOAD + i] = inc_buf[ML_POS_PAYLOAD + i];
    }

    /* queue message */
//    recorded_sysid = 0;
    ml_queue_msg(buf, sys_id);
}
/***************************************************************************/
void mavlink_lora_lib::ml_queue_msg_mission_request (unsigned short seq)
{
    /* id=40 */
    /* reference: mavlink/common/mavlink_msg_mission_request.h */
    unsigned char i, len;
    unsigned char *buf = (txbuf + txbuf_cnt);

    /* encode part of the header */
    buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_MISSION_REQUEST_INT_LEN;
    buf[ML_POS_SYS_ID] = recorded_sysid;
    buf[ML_POS_COMP_ID] = 0;
    buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_MISSION_REQUEST_INT;

    /* seq */
    buf[ML_POS_PAYLOAD + 0] = seq & 0xff;
    buf[ML_POS_PAYLOAD + 1] = (seq>>8) & 0xff;

    /* system_id (target) */
    buf[ML_POS_PAYLOAD + 2] = MAV_SYS_ID_UA; /* UA is the target system */

    /* component_id (target) */
    buf[ML_POS_PAYLOAD + 3] = 0; /* target component */

    /* queue message */
    ml_queue_msg(buf, recorded_sysid);
}
/***************************************************************************/
void mavlink_lora_lib::ml_queue_msg_mission_request_list()
{
    /* id=43 */
    /* reference: mavlink/common/mavlink_msg_mission_request_list.h */
    unsigned char i, len;
    unsigned char *buf = (txbuf + txbuf_cnt);

    /* encode part of the header */
    buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN;
    buf[ML_POS_SYS_ID] = recorded_sysid;
    buf[ML_POS_COMP_ID] = 0;
    buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_MISSION_REQUEST_LIST;

    /* system_id (target) */
    buf[ML_POS_PAYLOAD + 0] = MAV_SYS_ID_UA; /* UA is the target system */

    /* component_id (target) */
    buf[ML_POS_PAYLOAD + 1] = 0; /* target component */

    /* queue message */
    ml_queue_msg(buf, recorded_sysid);
}
/***************************************************************************/
void mavlink_lora_lib::ml_queue_msg_mission_item_int (float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z, unsigned short seq, unsigned short command, unsigned char frame, unsigned char current, unsigned char autocontinue)
{
    /* id=73 */
    /* reference: mavlink/common/mavlink_msg_mission_item_int.h */
    unsigned char i, len;
    unsigned char *buf = (txbuf + txbuf_cnt);
    unsigned char *pv;

    /* encode part of the header */
    buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN;
    buf[ML_POS_SYS_ID] = recorded_sysid;
    buf[ML_POS_COMP_ID] = 0;
    buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_MISSION_ITEM_INT;

    /* parameters */
    pv = (unsigned char *) &param1;
    buf[ML_POS_PAYLOAD + 0] = pv[0];
    buf[ML_POS_PAYLOAD + 1] = pv[1];
    buf[ML_POS_PAYLOAD + 2] = pv[2];
    buf[ML_POS_PAYLOAD + 3] = pv[3];

    pv = (unsigned char *) &param2;
    buf[ML_POS_PAYLOAD + 4] = pv[0];
    buf[ML_POS_PAYLOAD + 5] = pv[1];
    buf[ML_POS_PAYLOAD + 6] = pv[2];
    buf[ML_POS_PAYLOAD + 7] = pv[3];

    pv = (unsigned char *) &param3;
    buf[ML_POS_PAYLOAD + 8] = pv[0];
    buf[ML_POS_PAYLOAD + 9] = pv[1];
    buf[ML_POS_PAYLOAD + 10] = pv[2];
    buf[ML_POS_PAYLOAD + 11] = pv[3];

    pv = (unsigned char *) &param4;
    buf[ML_POS_PAYLOAD + 12] = pv[0];
    buf[ML_POS_PAYLOAD + 13] = pv[1];
    buf[ML_POS_PAYLOAD + 14] = pv[2];
    buf[ML_POS_PAYLOAD + 15] = pv[3];

    /* coordinates */
    pv = (unsigned char *) &x;
    buf[ML_POS_PAYLOAD + 16] = pv[0];
    buf[ML_POS_PAYLOAD + 17] = pv[1];
    buf[ML_POS_PAYLOAD + 18] = pv[2];
    buf[ML_POS_PAYLOAD + 19] = pv[3];

    pv = (unsigned char *) &y;
    buf[ML_POS_PAYLOAD + 20] = pv[0];
    buf[ML_POS_PAYLOAD + 21] = pv[1];
    buf[ML_POS_PAYLOAD + 22] = pv[2];
    buf[ML_POS_PAYLOAD + 23] = pv[3];

    pv = (unsigned char *) &z;
    buf[ML_POS_PAYLOAD + 24] = pv[0];
    buf[ML_POS_PAYLOAD + 25] = pv[1];
    buf[ML_POS_PAYLOAD + 26] = pv[2];
    buf[ML_POS_PAYLOAD + 27] = pv[3];

    /* sequence */
    buf[ML_POS_PAYLOAD + 28] = seq & 0xff;
    buf[ML_POS_PAYLOAD + 29] = (seq>>8) & 0xff;

    /* command */
    buf[ML_POS_PAYLOAD + 30] = command & 0xff;
    buf[ML_POS_PAYLOAD + 31] = (command>>8) & 0xff;

    /* system_id (target) */
    buf[ML_POS_PAYLOAD + 32] = MAV_SYS_ID_UA; /* UA is the target system */

    /* component_id (target) */
    buf[ML_POS_PAYLOAD + 33] = 0; /* target component, 0 equals ALL */

    /* Frame */
    buf[ML_POS_PAYLOAD + 34] = frame;

    /* Current */
    buf[ML_POS_PAYLOAD + 35] = current;

    /* autocontinue */
    buf[ML_POS_PAYLOAD + 36] = autocontinue;

    /* queue message */
    ml_queue_msg(buf, recorded_sysid);
}
/***************************************************************************/
void mavlink_lora_lib::ml_queue_msg_mission_count (unsigned short count)
{
    /* id=44 */
    /* reference: mavlink/common/mavlink_msg_mission_count.h */
    unsigned char i, len;
    unsigned char *buf = (txbuf + txbuf_cnt);

    /* encode part of the header */
    buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_MISSION_COUNT_LEN;
    buf[ML_POS_SYS_ID] = recorded_sysid;
    buf[ML_POS_COMP_ID] = 0;
    buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_MISSION_COUNT;

    /* count (number of mission items to upload */
    buf[ML_POS_PAYLOAD + 0] = count & 0xff;
    buf[ML_POS_PAYLOAD + 1] = (count>>8) & 0xff;

    /* system_id (target) */
    buf[ML_POS_PAYLOAD + 2] = MAV_SYS_ID_UA; /* UA is the target system */

    /* component_id (target) */
    buf[ML_POS_PAYLOAD + 3] = 0; /* target component */

    /* queue message */
    ml_queue_msg(buf, recorded_sysid);
}
/***************************************************************************/
void mavlink_lora_lib::ml_queue_msg_mission_clear_all()
{
    /* id=45 */
    /* reference: mavlink/common/mavlink_msg_mission_clear_all.h */
    unsigned char i, len;
    unsigned char *buf = (txbuf + txbuf_cnt);

    /* encode part of the header */
    buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN;
    buf[ML_POS_SYS_ID] = recorded_sysid;
    buf[ML_POS_COMP_ID] = 0;
    buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_MISSION_CLEAR_ALL;

    /* system_id (target) */
    buf[ML_POS_PAYLOAD + 0] = MAV_SYS_ID_UA; /* UA is the target system */

    /* component_id (target) */
    buf[ML_POS_PAYLOAD + 1] = 0; /* target component */

    /* queue message */
    ml_queue_msg(buf, recorded_sysid);
}
/***************************************************************************/
void mavlink_lora_lib::ml_queue_msg_mission_ack(uint8_t result)
{
    /* id=47 */
    /* reference: mavlink/common/mavlink_msg_mission_ack.h */
    unsigned char i, len;
    unsigned char *buf = (txbuf + txbuf_cnt);

    /* encode part of the header */
    buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_MISSION_ACK_LEN;
    buf[ML_POS_SYS_ID] = recorded_sysid;
    buf[ML_POS_COMP_ID] = 0;
    buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_MISSION_ACK;

    /* system_id (target) */
    buf[ML_POS_PAYLOAD + 0] = MAV_SYS_ID_UA; /* UA is the target system */

    /* component_id (target) */
    buf[ML_POS_PAYLOAD + 1] = 0; /* target component */

    /* type */
    buf[ML_POS_PAYLOAD + 2] = result; /* type (MAVLINK_MISSION_RESULT enum) */

    /* queue message */
    ml_queue_msg(buf, recorded_sysid);
}
/***************************************************************************/
void mavlink_lora_lib::ml_queue_msg_command_long(unsigned short cmd_id, float param1, float param2, float param3, float param4, float param5, float param6, float param7, unsigned int confirmation )
{
    /* id=76 */
    /* reference: mavlink/common/mavlink_msg_command_long */
    unsigned char *buf = (txbuf + txbuf_cnt);
    unsigned char *pv;

    /* encode part of the header */
    buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_COMMAND_LONG_LEN;
    buf[ML_POS_SYS_ID] = recorded_sysid;
    buf[ML_POS_COMP_ID] = 0;
    buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_COMMAND_LONG;

    /* param_value */
    pv = (unsigned char *) &param1;
    buf[ML_POS_PAYLOAD + 0]  = pv[0];
    buf[ML_POS_PAYLOAD + 1]  = pv[1];
    buf[ML_POS_PAYLOAD + 2]  = pv[2];
    buf[ML_POS_PAYLOAD + 3]  = pv[3];

    pv = (unsigned char *) &param2;
    buf[ML_POS_PAYLOAD + 4]  = pv[0];
    buf[ML_POS_PAYLOAD + 5]  = pv[1];
    buf[ML_POS_PAYLOAD + 6]  = pv[2];
    buf[ML_POS_PAYLOAD + 7]  = pv[3];

    pv = (unsigned char *) &param3;
    buf[ML_POS_PAYLOAD + 8]  = pv[0];
    buf[ML_POS_PAYLOAD + 9]  = pv[1];
    buf[ML_POS_PAYLOAD + 10]  = pv[2];
    buf[ML_POS_PAYLOAD + 11]  = pv[3];

    pv = (unsigned char *) &param4;
    buf[ML_POS_PAYLOAD + 12] = pv[0];
    buf[ML_POS_PAYLOAD + 13] = pv[1];
    buf[ML_POS_PAYLOAD + 14] = pv[2];
    buf[ML_POS_PAYLOAD + 15] = pv[3];

    pv = (unsigned char *) &param5;
    buf[ML_POS_PAYLOAD + 16] = pv[0];
    buf[ML_POS_PAYLOAD + 17] = pv[1];
    buf[ML_POS_PAYLOAD + 18] = pv[2];
    buf[ML_POS_PAYLOAD + 19] = pv[3];

    pv = (unsigned char *) &param6;
    buf[ML_POS_PAYLOAD + 20] = pv[0];
    buf[ML_POS_PAYLOAD + 21] = pv[1];
    buf[ML_POS_PAYLOAD + 22] = pv[2];
    buf[ML_POS_PAYLOAD + 23] = pv[3];

    pv = (unsigned char *) &param7;
    buf[ML_POS_PAYLOAD + 24] = pv[0];
    buf[ML_POS_PAYLOAD + 25] = pv[1];
    buf[ML_POS_PAYLOAD + 26] = pv[2];
    buf[ML_POS_PAYLOAD + 27] = pv[3];

    /* command_id */
    buf[ML_POS_PAYLOAD + 28] = cmd_id & 0xff;
    buf[ML_POS_PAYLOAD + 29] = (cmd_id>>8) & 0xff;

    /* system_id (target) */
    buf[ML_POS_PAYLOAD + 30] = MAV_SYS_ID_UA; /* UA is the target system */

    /* component_id (target) */
    buf[ML_POS_PAYLOAD + 31] = 0; /* target component */

    /* confirmation */
    buf[ML_POS_PAYLOAD + 32] = confirmation;

    /* queue message */
    ml_queue_msg(buf, recorded_sysid);
}
void mavlink_lora_lib::ml_queue_msg_command_ack(unsigned short cmd_id, unsigned char result) {
    /* id=77 */
    /* reference: mavlink/common/command_ack */
    unsigned char *buf = (txbuf + txbuf_cnt);
    unsigned char *pv;

    /* encode part of the header */
    buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_COMMAND_ACK_LEN;
    buf[ML_POS_SYS_ID] = recorded_sysid;
    buf[ML_POS_COMP_ID] = 0;
    buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_COMMAND_ACK;

    /* command_id */
    buf[ML_POS_PAYLOAD + 0] = cmd_id & 0xff;
    buf[ML_POS_PAYLOAD + 1] = (cmd_id>>8) & 0xff;

    /* result */
    buf[ML_POS_PAYLOAD + 2] = result;

    /* queue message */
    ml_queue_msg(buf, recorded_sysid);
}
/***************************************************************************/
void mavlink_lora_lib::ml_queue_msg_heartbeat(unsigned char type, unsigned char autopilot, unsigned char base_mode, unsigned long custom_mode, unsigned char system_status, unsigned char system_id)
{
    /* id=0 */
    /* reference: mavlink/common/mavlink_msg_heartbeat */
    unsigned char *buf = (txbuf + txbuf_cnt);
    unsigned char *cm;

    /* encode part of the header */
    buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_HEARTBEAT_LEN;
    buf[ML_POS_SYS_ID] = system_id; //send GCS system_id => 255
    buf[ML_POS_COMP_ID] = 0;
    buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_HEARTBEAT;

    /* custom mode */
    cm = (unsigned char *) &custom_mode;
    buf[ML_POS_PAYLOAD + 0] = cm[0];
    buf[ML_POS_PAYLOAD + 1] = cm[1];
    buf[ML_POS_PAYLOAD + 2] = cm[2];
    buf[ML_POS_PAYLOAD + 3] = cm[3];

    /* type. 6 = MAV_TYPE_GCS */
    buf[ML_POS_PAYLOAD + 4] = type; //6 == ground control station

    /* autopilot */
    buf[ML_POS_PAYLOAD + 5] = autopilot; //8 = INVALID - Not a drone, but gcs software

    /* base_mode */
    buf[ML_POS_PAYLOAD + 6] = base_mode; //1 = custom mode

    /* system_status */
    buf[ML_POS_PAYLOAD + 7] = system_status; //4 = ACTIVE_STATE, GCS is active. Required to get status messages

    /* System id */
    buf[ML_POS_PAYLOAD + 8] = system_id; //send GCS system_id => 255

    /* queue message */
    ml_queue_msg(buf, recorded_sysid);
}
/***************************************************************************/
unsigned long mavlink_lora_lib::ml_messages_sent()
{
    return msg_tx_cnt;
}
unsigned long mavlink_lora_lib::ml_messages_received()
{
    return msg_rx_cnt;
}
unsigned long mavlink_lora_lib::ml_messages_crc_error()
{
    return msg_crc_err;
}
unsigned long mavlink_lora_lib::ml_messages_lost(unsigned char sys_id)
{
    return lost_messages_map[sys_id].lostMessages;
}
unsigned long mavlink_lora_lib::millis()
{
    struct timeval te;
    gettimeofday(&te, NULL); /* get current time */

    if (secs_init == 0)
    {
        secs_init = te.tv_sec;
    }

    return ((unsigned long) (te.tv_sec - secs_init)*1000 + te.tv_usec/1000);
}

/***************************************************************************/
mavlink_sys_status_t mavlink_lora_lib::ml_unpack_msg_sys_status (unsigned char *payload)
{
    /* id=1 */
    /* mavlink/common/mavlink_msg_sys_status.h */
    mavlink_sys_status_t sys_status;

    sys_status.load = payload[12] | (payload[13] << 8);
    sys_status.voltage_battery = payload[14] | (payload[15] << 8);
    sys_status.current_battery = payload[16] | (payload[17] << 8);
    sys_status.battery_remaining = payload[30];
    sys_status.errors_comm = payload[20] | (payload[21] << 8);

    return sys_status;
}
mavlink_gps_raw_int_t mavlink_lora_lib::ml_unpack_msg_gps_raw_int (unsigned char *payload)
{
    /* id=24 */
    /* mavlink/common/mavlink_gps_raw_int.h */
    mavlink_gps_raw_int_t gri;
    uint64_t *ui64p;
    int32_t *i32p;
    uint8_t *ui8p;
    uint16_t *ui16p;

    ui64p = (uint64_t *) (payload + 0);
    gri.time_usec = *ui64p;
    i32p = (int32_t *) (payload + 8);
    gri.lat = *i32p;
    i32p = (int32_t *) (payload + 12);
    gri.lon = *i32p;
    i32p = (int32_t *) (payload + 16);
    gri.alt = *i32p;
    ui16p = (uint16_t *) (payload + 20);
    gri.eph = *ui16p;
    ui16p = (uint16_t *) (payload + 22);
    gri.epv = *ui16p;
    ui16p = (uint16_t *) (payload + 24);
    gri.vel = *ui16p;
    ui16p = (uint16_t *) (payload + 26);
    gri.cog = *ui16p;
    ui8p = (uint8_t *) (payload + 28);
    gri.fix_type = *ui8p;
    ui8p = (uint8_t *) (payload + 29);
    gri.satellites_visible = *ui8p;

    return gri;
}
mavlink_attitude_t mavlink_lora_lib::ml_unpack_msg_attitude (unsigned char *payload)
{
    /* id=30 */
    /* mavlink/common/mavlink_attitude.h */
    mavlink_attitude_t atti;
    uint32_t *ui32p;
    float *fp;

    ui32p = (uint32_t *) (payload + 0);
    atti.time_boot_ms = *ui32p;
    fp = (float *) (payload + 4);
    atti.roll = *fp;
    fp = (float *) (payload + 8);
    atti.pitch = *fp;
    fp = (float *) (payload + 12);
    atti.yaw = *fp;

    return atti;
}
mavlink_global_position_int_t mavlink_lora_lib::ml_unpack_msg_global_position_int (unsigned char *payload)
{
    /* id=33 */
    /* mavlink/common/mavlink_global_position_int.h */
    mavlink_global_position_int_t gpi;
    int32_t *i32p;
    int16_t *i16p;
    uint16_t *ui16p;

    i32p = (int32_t *) (payload + 0);
    gpi.time_boot_ms = *i32p;
    i32p = (int32_t *) (payload + 4);
    gpi.lat = *i32p;
    i32p = (int32_t *) (payload + 8);
    gpi.lon = *i32p;
    i32p = (int32_t *) (payload + 12);
    gpi.alt = *i32p;
    i32p = (int32_t *) (payload + 16);
    gpi.relative_alt = *i32p;
    i16p = (int16_t *) (payload + 20);
    gpi.vx = *i16p;
    i16p = (int16_t *) (payload + 22);
    gpi.vx = *i16p;
    i16p = (int16_t *) (payload + 24);
    gpi.vx = *i16p;
    ui16p = (uint16_t *) (payload + 26);
    gpi.hdg = *ui16p;

    return gpi;
}
mavlink_mission_item_t mavlink_lora_lib::ml_unpack_msg_mission_item (unsigned char *payload)
{
    /* id=39 */
    /* mavlink/common/mavlink_msg_mission_item.h */
    mavlink_mission_item_t item;
    float *fp;

    fp = (float *) (payload + 0);
    item.param1 = *fp;
    fp = (float *) (payload + 4);
    item.param2 = *fp;
    fp = (float *) (payload + 8);
    item.param3 = *fp;
    fp = (float *) (payload + 12);
    item.param4 = *fp;
    fp = (float *) (payload + 16);
    item.x = *fp;
    fp = (float *) (payload + 20);
    item.y = *fp;
    fp = (float *) (payload + 24);
    item.z = *fp;

    item.seq = payload[28] | (payload[29] << 8);
    item.command = payload[30] | (payload[31] << 8);
    item.target_system = payload[32];
    item.target_component = payload[33];
    item.frame = payload[34];
    item.current = payload[35];
    item.autocontinue = payload[36];

    return item;
}
mavlink_mission_item_int_t mavlink_lora_lib::ml_unpack_msg_mission_item_int (unsigned char *payload)
{
    /* id=73 */
    /* mavlink/common/mavlink_msg_mission_item_int.h */
    mavlink_mission_item_int_t item;
    float *fp;
    uint32_t *ui32p;

    fp = (float *) (payload + 0);
    item.param1 = *fp;
    fp = (float *) (payload + 4);
    item.param2 = *fp;
    fp = (float *) (payload + 8);
    item.param3 = *fp;
    fp = (float *) (payload + 12);
    item.param4 = *fp;
    ui32p = (uint32_t *) (payload + 16);
    item.x = *ui32p;
    ui32p = (uint32_t *) (payload + 20);
    item.y = *ui32p;
    fp = (float *) (payload + 24);
    item.z = *fp;

    item.seq = payload[28] | (payload[29] << 8);
    item.command = payload[30] | (payload[31] << 8);
    item.target_system = payload[32];
    item.target_component = payload[33];
    item.frame = payload[34];
    item.current = payload[35];
    item.autocontinue = payload[36];

    return item;
}
mavlink_mission_request_int_t mavlink_lora_lib::ml_unpack_msg_mission_request_int (unsigned char *payload)
{
    /* id=51 */
    /* mavlink/common/mavlink_msg_mission_request_int.h */
    mavlink_mission_request_int_t item;
    uint16_t *ui16p;

    ui16p = (uint16_t *) (payload + 0);
    item.seq = *ui16p;
    item.target_system = payload[2];
    item.target_component = payload[3];

    return item;
}
mavlink_heartbeat_t mavlink_lora_lib::ml_unpack_msg_heartbeat (unsigned char *payload)
{
    /* id=0 */
    /* mavlink/common/heartbeat.h */
    mavlink_heartbeat_t item;
    uint32_t *ui32p;

    ui32p = (uint32_t *) (payload + 0);
    item.custom_mode = *ui32p;

    item.type = payload[4];
    item.autopilot = payload[5];
    item.base_mode = payload[6];
    item.system_status = payload[7];
    item.system_id = payload[8];

    return item;
}
mavlink_mission_ack_t mavlink_lora_lib::ml_unpack_msg_mission_ack (unsigned char *payload)
{
    /* id=47 */
    /* mavlink/common/mavlink_mission_ack */
    mavlink_mission_ack_t ack;

    ack.target_system = payload[0];
    ack.target_component = payload[1];
    ack.type = payload[2];

    return ack;
}
unsigned short mavlink_lora_lib::ml_unpack_msg_mission_current (unsigned char *payload)
{
    /* id=42 */
    /* mavlink/common/mavlink_msg_mission_current.h */
    return payload[0] | (payload[1] << 8);
}
mavlink_mission_count_t mavlink_lora_lib::ml_unpack_msg_mission_count (unsigned char *payload)
{
    /* id=44 */
    /* mavlink/common/mavlink_msg_mission_count.h */
    mavlink_mission_count_t item;

    item.count = payload[0] | (payload[1] << 8);
    item.target_system = payload[2];
    item.target_component = payload[3];

    return item;
}
mavlink_mission_partial_write_list_t mavlink_lora_lib::ml_unpack_msg_mission_partial_write_list(unsigned char *payload)
{
    /* id=38 */
    /* mavlink/common/mavlink_msg_mission_partial_write_list */
    mavlink_mission_partial_write_list_t item;

    item.start_index = payload[0] | (payload[1] << 8);
    item.end_index = payload[2] | (payload[3] << 8);
    item.target_system = payload[4];
    item.target_component = payload[5];

    return item;
}
mavlink_statustext_t mavlink_lora_lib::ml_unpack_msg_statustext (unsigned char *payload)
{
    /* id=253 */
    /* mavlink/common/mavlink_msg_statustext.h */
    mavlink_statustext_t statustext;

    statustext.severity = payload[0];
    memcpy (statustext.text, payload+1, 50);

    return statustext;
}
mavlink_command_ack_t mavlink_lora_lib::ml_unpack_msg_command_ack (unsigned char *payload)
{
    /* id=77 */
    /* mavlink/common/mavlink_command_ack */
    mavlink_command_ack_t ack;

    ack.command = payload[0] | (payload[1] << 8);
    ack.result = payload[2];

    return ack;
}
mavlink_radio_status_t mavlink_lora_lib::ml_unpack_msg_radio_status (unsigned char *payload)
{
    /* id=109 */
    /* mavlink/common/mavlink_radio_status */
    mavlink_radio_status_t radioStatus;
    uint16_t *ui16p;

    ui16p = (uint16_t *) (payload + 0);
    radioStatus.rxerrors = *ui16p;

    ui16p = (uint16_t *) (payload + 2);
    radioStatus.fixed = *ui16p;

    radioStatus.rssi = payload[4];
    radioStatus.remrssi = payload[5];
    radioStatus.txbuf = payload[6];
    radioStatus.noise = payload[7];
    radioStatus.remnoise = payload[8];

    return radioStatus;
}
mavlink_system_time_t mavlink_lora_lib::ml_unpack_msg_system_time(unsigned char *payload) {
    /* id=2 */
    /* mavlink/common/system_time */
    mavlink_system_time_t systemTime;

    uint32_t *ui32p;
    uint64_t *ui64p;

    ui64p = (uint64_t *) (payload + 0);
    ui32p = (uint32_t *) (payload + 8);

    systemTime.time_unix_usec = *ui64p;
    systemTime.time_boot_ms = *ui32p;

    return systemTime;
}

mavlink_altitude_t mavlink_lora_lib::ml_unpack_msg_altitude(unsigned char *payload) {
    /* id=141 */
    /* mavlink/common/altitude */
    mavlink_altitude_t altitude;

    uint64_t *ui64p;
    float *fp;

    ui64p = (uint64_t *) (payload + 0);
    altitude.time_usec = *ui64p;

    fp = (float * ) (payload + 8);
    altitude.altitude_monotonic = *fp;

    fp = (float * ) (payload + 12);
    altitude.altitude_amsl = *fp;

    fp = (float * ) (payload + 16);
    altitude.altitude_local = *fp;

    fp = (float * ) (payload + 20);
    altitude.altitude_relative = *fp;

    fp = (float * ) (payload + 24);
    altitude.altitude_terrain = *fp;

    fp = (float * ) (payload + 28);
    altitude.bottom_clearence = *fp;

    return altitude;
}

mavlink_battery_status_t mavlink_lora_lib::ml_unpack_msg_battery_status(unsigned char *payload) {
    /* id=147 */
    /* mavlink/common/battery_status */
    mavlink_battery_status_t batteryStatus;

    int32_t *i32p;
    float *fp;
    int16_t *i16p;
    uint16_t *ui16p;

    i32p = (int32_t *) (payload + 0);
    batteryStatus.current_consumed = *i32p;

    i32p = (int32_t *) (payload + 4);
    batteryStatus.energy_consumed = *i32p;

    i16p = (int16_t *) (payload + 8);
    batteryStatus.temperature = *i16p;

    //VOLTAGES
    for ( int i = 0; i < 10; i++)
    {
        ui16p = (uint16_t *) (payload + (10 + (i * 2) ));
        batteryStatus.voltages[i] = *ui16p;
    }

    i16p = (int16_t *) (payload + 30);
    batteryStatus.current_battery = *i16p;

    batteryStatus.id = payload[32];
    batteryStatus.battery_function = payload[33];
    batteryStatus.type = payload[34];
    batteryStatus.battery_remaining = payload[35];

    return batteryStatus;
}

mavlink_vibration_t mavlink_lora_lib::ml_unpack_msg_vibration(unsigned char *payload) {
    /* id=241 */
    /* mavlink/common/vibration */
    mavlink_vibration_t vibration;

    uint64_t *ui64p;
    uint32_t *ui32p;
    float *fp;

    ui64p = (uint64_t *) (payload + 0);
    vibration.time_usec = *ui64p;

    fp = (float * ) (payload + 8);
    vibration.vibration_x = *fp;

    fp = (float * ) (payload + 12);
    vibration.vibration_y = *fp;

    fp = (float * ) (payload + 16);
    vibration.vibration_z = *fp;

    ui32p = (uint32_t *) (payload + 20);
    vibration.clipping_0 = *ui32p;

    ui32p = (uint32_t *) (payload + 24);
    vibration.clipping_1 = *ui32p;

    ui32p = (uint32_t *) (payload + 28);
    vibration.clipping_2 = *ui32p;

    return vibration;
}

mavlink_extended_sys_status_t mavlink_lora_lib::ml_unpack_msg_extended_sys_status(unsigned char *payload) {
    /* id=245 */
    /* mavlink/common/extended_sys_status */
    mavlink_extended_sys_status_t extendedSysStatus;

    extendedSysStatus.vtol_state = payload[0];
    extendedSysStatus.landed_state = payload[1];

    return extendedSysStatus;
}
mavlink_command_long_t mavlink_lora_lib::ml_unpack_msg_command_long(unsigned char *payload) {
    /* id=76 */
    /* mavlink/common/command_long */
    mavlink_command_long_t command_long;

    uint16_t *ui16p;
    float *fp;

    fp = (float * ) (payload + 0);
    command_long.param1 = *fp;

    fp = (float * ) (payload + 4);
    command_long.param2 = *fp;

    fp = (float * ) (payload + 8);
    command_long.param3 = *fp;

    fp = (float * ) (payload + 12);
    command_long.param4 = *fp;

    fp = (float * ) (payload + 16);
    command_long.param5 = *fp;

    fp = (float * ) (payload + 20);
    command_long.param6 = *fp;

    fp = (float * ) (payload + 24);
    command_long.param7 = *fp;

    ui16p = (uint16_t *) (payload + 28);
    command_long.command = *ui16p;

    command_long.target_system = payload[30];
    command_long.target_component = payload[31];
    command_long.confirmation = payload[32];

    return command_long;
}
mavlink_home_position_t mavlink_lora_lib::ml_unpack_msg_home_position(unsigned char *payload) {
    /* id=242 */
    /* mavlink/common/home_position */
    mavlink_home_position_t home_position;

    int32_t *i32p;
    float *fp;

    i32p = (int32_t * ) (payload + 0);
    home_position.latitude = *i32p;

    i32p = (int32_t * ) (payload + 4);
    home_position.longitude = *i32p;

    i32p = (int32_t * ) (payload + 8);
    home_position.altitude = *i32p;

    fp = (float * ) (payload + 12);
    home_position.x = *fp;

    fp = (float * ) (payload + 16);
    home_position.y = *fp;

    fp = (float * ) (payload + 20);
    home_position.z = *fp;

    for ( int i = 0; i < 4; i++)
    {
        fp = (float *) (payload + (24 + (i * 4) ));
        home_position.q[i] = *fp;
    }

    fp = (float * ) (payload + 40);
    home_position.approach_x = *fp;

    fp = (float * ) (payload + 44);
    home_position.approach_y = *fp;

    fp = (float * ) (payload + 48);
    home_position.approach_z = *fp;

    return home_position;
}

/***************************************************************************/
void mavlink_lora_lib::ml_tx_update()
{
    int bytes_written = ser_send(ser_ref, txbuf, txbuf_cnt);
    txbuf_cnt = 0;
}

short mavlink_lora_lib::ml_rx_update(unsigned long now, unsigned char *rxbuf_new, short rxbuf_new_cnt)
{
    char result = 0;
    short i, j, count;
    unsigned char c;

    /* check for buffer owerflow */
    if (rxbuf_cnt + rxbuf_new_cnt > RX_BUF_SIZE)
    {
        rxbuf_cnt = 0;
        result = -1;
        msg_buf_overflow++;
        if (debug)
            cout << "Buffer overflow" << endl;
    }
    else
    {
        short seek_from = rxbuf_cnt;
        char maybe_more = 1;
        txbuf_cnt = 0;

        /* add new bytes to buffer */
        /*for (i=0; i<rxbuf_new_cnt; i++)
            rxbuf[rxbuf_cnt++] = rxbuf_new[i];*/
        memcpy (rxbuf+rxbuf_cnt , rxbuf_new, rxbuf_new_cnt);
        rxbuf_cnt += rxbuf_new_cnt;

        while (maybe_more == 1)
        {
            maybe_more = 0;
            if (msg_begin < 0) /* try to find a packet start */
            {
                for (i=seek_from; i<rxbuf_cnt; i++)
                {
                    if (rxbuf[i] == ML_NEW_PACKET_IDENT_V10 && msg_begin < 0)
                        msg_begin = i;
                }
            }

            /* if we have found a packet start and the packet len > minimum */
            if (msg_begin >= 0 && rxbuf_cnt >= msg_begin + 8)
            {
                short payload_len = rxbuf[msg_begin + ML_POS_PAYLOAD_LEN];
                short msg_next = msg_begin + payload_len + 8; /* actually beginning of next */

                /* if we have a complete packet */
                if (rxbuf_cnt >= msg_next)
                {
                    unsigned char crc_ok;
                    unsigned char msg_id = rxbuf[msg_begin + ML_POS_MSG_ID];
                    unsigned char msg_seq = rxbuf[msg_begin + ML_POS_PACKET_SEQ];
                    unsigned char sys_id = rxbuf[msg_begin + ML_POS_SYS_ID];

                    /* if the checksum is valid */
                    unsigned char crc_lsb = rxbuf[msg_begin + payload_len + 6];
                    unsigned char crc_msb = rxbuf[msg_begin + payload_len + 7];
                    unsigned short crc = crc_calculate(rxbuf+msg_begin+1, payload_len+5);
                    unsigned char crc_extra = MAVLINK_MESSAGE_CRCS_V1[msg_id];
                    crc_accumulate(crc_extra, &crc);
                    crc_ok = ((crc & 0xff) == crc_lsb && (crc >> 8) == crc_msb);

                    unsigned char payload_len = rxbuf[msg_begin + ML_POS_PAYLOAD_LEN];
                    unsigned char seq = rxbuf[msg_begin + ML_POS_PACKET_SEQ];

                    if (crc_ok)
                    {
                        msg_rx_cnt++;
                        do_send_msg = 1;

                        // get lost msg map for this sys_id
                        auto& lost_msgs_ref = lost_messages_map[sys_id];

                        /* if first time record the sys_id */
                        if (msg_id == 0 && recorded_sysid == 0)
                        {
                            recorded_sysid = sys_id;
                        }

                        // set next sequence expected for first time
                        if (lost_msgs_ref.last_seq == 0)
                            lost_msgs_ref.last_seq = msg_seq;

                        // Manage check for lost messages
                        if (msg_seq > lost_msgs_ref.last_seq + 1)
                        {
                            // we have lost packages
                            auto numLost = msg_seq - (lost_msgs_ref.last_seq + 1);
                            lost_msgs_ref.lostMessages += numLost;
                        }

                        // save current msg as last msg
                        lost_msgs_ref.last_seq = msg_seq;

                        /* check if param or mission sequence is ongoing */
                        if (do_send_msg == 1)
                        {
                            if (msg_id==20 || msg_id==21 || msg_id==22 || msg_id==23) /* param msgs */
                            {
                                param_mission_tout = now + PARAM_TOUT;
                            }
                            else if (msg_id==37 || msg_id==39 || msg_id==40 || msg_id==43 || msg_id==44 || msg_id==47) /* mission item transactions */
                            {
                                param_mission_tout = now + MISSION_TOUT;
                            }
                            else if (msg_id != 0)  /* heartbeat must get through */
                            {
                                if (now < param_mission_tout)
                                    do_send_msg = 0;
                            }
                        }

                        /* handle packet */
                        if (do_send_msg == 1)
                        {
                            result ++;

                            parse_msg_callback(rxbuf + msg_begin);

                            /*printf ("%ld accepted %d\n", ms, msg_id);*/
                        }
                        /*else printf ("%ld dropped %d\n", ms, msg_id);  */
                    }
                    else
                    {
                        msg_crc_err++;
                        if (debug && msg_crc_err != 1) /* dischard first CRC error which usually occurs at startup */
                        {
                            printf ("CRC error (len %d): ", (msg_next - msg_begin));
                            for (i=msg_begin; i<msg_next; i++)
                            {
                                printf ("%03d ", rxbuf[i]);
                            }
                            printf ("\n");

                        }
                    }

                    /* remove packet from rxbuf */
                    for (i=msg_next, j=0; i<rxbuf_cnt; i++, j++)
                    {
                        rxbuf[j] = rxbuf[i];
                    }
                    rxbuf_cnt -= msg_next;
                    /* printf ("rxbuf_cnt_after %d\n", rxbuf_cnt); */

                    msg_begin = -1;
                    seek_from = 0;
                    maybe_more = 1;
                    /* printf ("repeat\n");  */
                }
            }
        }
    }
    return result;
}
