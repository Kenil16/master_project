//
// Created by Crow on 25-03-2019.
//

#ifndef MAVLINK_BRIDGE_MAVLINK_MESSAGES_HPP
#define MAVLINK_BRIDGE_MAVLINK_MESSAGES_HPP

/* defines */

/* mavlink message content */
#define ML_NEW_PACKET_IDENT_V10	0xfe /* MavLink v1.0 */
#define ML_NEW_PACKET_IDENT_V20	0xfd /* MavLink v2.0 */

/* mavlink system id */
#define MAV_SYS_ID_ALL 0
#define MAV_SYS_ID_UA 1
#define MAV_SYS_ID_GCS 255

/* mavlink component id */
/*#define MAV_COMP_ID_ALL 0 */

/* mavlink message format */
#define ML_POS_IDENT 0
#define ML_POS_PAYLOAD_LEN 1
#define ML_POS_PACKET_SEQ 2
#define ML_POS_SYS_ID 3
#define ML_POS_COMP_ID 4
#define ML_POS_MSG_ID 5
#define ML_POS_PAYLOAD 6

/* mavlink message id's /*
/* needed because we are not including the official mavlink headers */

#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9

#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_SYS_STATUS_LEN 31

#define MAVLINK_MSG_ID_SYSTEM_TIME 2
#define MAVLINK_MSG_ID_SYSTEM_TIME_LEN 6

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ 20
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN 20

#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST 21
#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST_LEN 2

#define MAVLINK_MSG_ID_PARAM_VALUE 22
#define MAVLINK_MSG_ID_PARAM_VALUE_LEN 25

#define MAVLINK_MSG_ID_PARAM_SET 23
#define MAVLINK_MSG_ID_PARAM_SET_LEN 23

#define MAVLINK_MSG_ID_GPS_RAW_INT 24
#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN 30

#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_ATTITUDE_LEN 28

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION 31

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED 32

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN 28

#define MAVLINK_MSG_ID_RC_CHANNELS_SCALED 34

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 35

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW 36

#define MAVLINK_MSG_ID_RC_CHANNELS 64

#define MAVLINK_MSG_ID_MANUAL_CONTROL 69

#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE 70

#define MAVLINK_MSG_ID_VFR_HUD 74

#define MAVLINK_MSG_ID_ATTITUDE_TARGET 83

#define MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT 87

#define MAVLINK_MSG_ID_HIGHRES_IMU 105

#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET 140

#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST 37
#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN 6

#define MAVLINK_MSG_ID_MISSION_ITEM 39
#define MAVLINK_MSG_ID_MISSION_ITEM_LEN 37

#define MAVLINK_MSG_ID_MISSION_ITEM_INT 73
#define MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN 37

#define MAVLINK_MSG_ID_MISSION_REQUEST 40
#define MAVLINK_MSG_ID_MISSION_REQUEST_LEN 4

#define MAVLINK_MSG_ID_MISSION_REQUEST_INT 51
#define MAVLINK_MSG_ID_MISSION_REQUEST_INT_LEN 4

#define MAVLINK_MSG_ID_MISSION_CURRENT 42
#define MAVLINK_MSG_ID_MISSION_CURRENT_LEN 2

#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST 43
#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN 2

#define MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST 38
#define MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_LEN 6

#define MAVLINK_MSG_ID_MISSION_COUNT 44
#define MAVLINK_MSG_ID_MISSION_COUNT_LEN 4

#define MAVLINK_MSG_ID_MISSION_CLEAR_ALL 45
#define MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN 2

#define MAVLINK_MSG_ID_MISSION_ACK 47
#define MAVLINK_MSG_ID_MISSION_ACK_LEN 3

#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM 66
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN 6

#define MAVLINK_MSG_ID_STATUSTEXT 253
#define MAVLINK_MSG_ID_STATUSTEXT_LEN 51

#define MAVLINK_MSG_ID_COMMAND_LONG 76
#define MAVLINK_MSG_ID_COMMAND_LONG_LEN 33

#define MAVLINK_MSG_ID_COMMAND_ACK 77
#define MAVLINK_MSG_ID_COMMAND_ACK_LEN 3

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED 84
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN 53

#define MAVLINK_MSG_ID_RADIO_STATUS 109

#define MAVLINK_MSG_ID_ALTITUDE 141
#define MAVLINK_MSG_ID_ALTITUDE_LEN 32

#define MAVLINK_MSG_ID_BATTERY_STATUS 147
#define MAVLINK_MSG_ID_BATTERY_STATUS_LEN 36

#define MAVLINK_MSG_ID_VIBRATION 241
#define MAVLINK_MSG_ID_VIBRATION_LEN 32

#define MAVLINK_MSG_ID_HOME_POSITION 242
#define MAVLINK_MSG_ID_HOME_POSITION_LEN 52

#define MAVLINK_MSG_ID_EXTENDED_SYS_STATUS 245
#define MAVLINK_MSG_ID_EXTENDED_SYS_STATUS_LEN 2

/* Command longs IDs */
#define MAVLINK_MSG_ID_COMPONENT_ARM_DISARM 400
#define MAVLINK_MSG_ID_MISSION_START 300
#define MAVLINK_MSG_ID_DO_SET_MODE 176
#define MAVLINK_MSG_ID_NAV_TAKEOFF 22
#define MAVLINK_MSG_ID_NAV_GUIDED_ENABLE 92
#define MAVLINK_MSG_ID_NAV_LAND 21
#define MAVLINK_MSG_ID_SET_REPOSITION 192
#define MAVLINK_MSG_ID_DO_PAUSE_CONTINUE 193
#define MAVLINK_MSG_ID_SET_MESSAGE_INTERVAL 511
#define MAVLINK_MSG_ID_RETURN_TO_LAUNCH 20
#define MAVLINK_MSG_ID_GET_HOME_POSITION 410


#define MAVLINK_MSG_ID_PREFLIGHT_CALIBRATION 241

struct mavlink_sys_status_t {
    uint32_t onboard_control_sensors_present; /*< Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
    uint32_t onboard_control_sensors_enabled; /*< Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
    uint32_t onboard_control_sensors_health; /*< Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
    uint16_t load; /*< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000*/
    uint16_t voltage_battery; /*< Battery voltage, in millivolts (1 = 1 millivolt)*/
    int16_t current_battery; /*< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current*/
    uint16_t drop_rate_comm; /*< Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
    uint16_t errors_comm; /*< Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
    uint16_t errors_count1; /*< Autopilot-specific errors*/
    uint16_t errors_count2; /*< Autopilot-specific errors*/
    uint16_t errors_count3; /*< Autopilot-specific errors*/
    uint16_t errors_count4; /*< Autopilot-specific errors*/
    int8_t battery_remaining; /*< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery*/
};

struct mavlink_attitude_t {
    uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
    float roll; /*< Roll angle (rad, -pi..+pi)*/
    float pitch; /*< Pitch angle (rad, -pi..+pi)*/
    float yaw; /*< Yaw angle (rad, -pi..+pi)*/
    float rollspeed; /*< Roll angular speed (rad/s)*/
    float pitchspeed; /*< Pitch angular speed (rad/s)*/
    float yawspeed; /*< Yaw angular speed (rad/s)*/
};

struct mavlink_gps_raw_int_t {
    uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
    int32_t lat; /*< Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7*/
    int32_t lon; /*< Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7*/
    int32_t alt; /*< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.*/
    uint16_t eph; /*< GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
    uint16_t epv; /*< GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
    uint16_t vel; /*< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
    uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
    uint8_t fix_type; /*< See the GPS_FIX_TYPE enum.*/
    uint8_t satellites_visible; /*< Number of satellites visible. If unknown, set to 255*/
};

struct mavlink_global_position_int_t {
    uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
    int32_t lat; /*< Latitude, expressed as degrees * 1E7*/
    int32_t lon; /*< Longitude, expressed as degrees * 1E7*/
    int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
    int32_t relative_alt; /*< Altitude above ground in meters, expressed as * 1000 (millimeters)*/
    int16_t vx; /*< Ground X Speed (Latitude, positive north), expressed as m/s * 100*/
    int16_t vy; /*< Ground Y Speed (Longitude, positive east), expressed as m/s * 100*/
    int16_t vz; /*< Ground Z Speed (Altitude, positive down), expressed as m/s * 100*/
    uint16_t hdg; /*< Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
};

struct mavlink_mission_item_t { /* mavlink/common/mavlink_msg_mission_item.h */
    float param1; /*< PARAM1, see MAV_CMD enum*/
    float param2; /*< PARAM2, see MAV_CMD enum*/
    float param3; /*< PARAM3, see MAV_CMD enum*/
    float param4; /*< PARAM4, see MAV_CMD enum*/
    float x; /*< PARAM5 / local: x position, global: latitude*/
    float y; /*< PARAM6 / y position: global: longitude*/
    float z; /*< PARAM7 / z position: global: altitude (relative or absolute, depending on frame.*/
    uint16_t seq; /*< Sequence*/
    uint16_t command; /*< The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs*/
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
    uint8_t frame; /*< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h*/
    uint8_t current; /*< false:0, true:1*/
    uint8_t autocontinue; /*< autocontinue to next wp*/
};

struct mavlink_mission_item_int_t { /* mavlink/common/mavlink_msg_mission_item_int.h */
    float param1; /*< PARAM1, see MAV_CMD enum*/
    float param2; /*< PARAM2, see MAV_CMD enum*/
    float param3; /*< PARAM3, see MAV_CMD enum*/
    float param4; /*< PARAM4, see MAV_CMD enum*/
    uint32_t x; /*< PARAM5 / local: x position, global: latitude*/
    uint32_t y; /*< PARAM6 / y position: global: longitude*/
    float z; /*< PARAM7 / z position: global: altitude (relative or absolute, depending on frame.*/
    uint16_t seq; /*< Sequence*/
    uint16_t command; /*< The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs*/
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
    uint8_t frame; /*< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h*/
    uint8_t current; /*< false:0, true:1*/
    uint8_t autocontinue; /*< autocontinue to next wp*/
};

struct mavlink_mission_request_int_t { /* mavlink/common/mavlink_msg_mission_request_int.h */
    uint16_t seq; /*< Sequence*/
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
};

struct mavlink_mission_count_t { /* mavlink/common/mavlink_msg_mission_count.h */
    uint16_t count; /*< number of mission items in the sequence*/
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
};

struct mavlink_mission_clear_all_t { /* mavlink/common/mavlink_msg_mission_clear_all */
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
};

struct mavlink_mission_ack_t { /* mavlink/common/mavlink_msg_mission_ack.h */
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
    uint8_t type; /*< Mission Result, check enum: MAV_MISSION_RESULT */
};

struct mavlink_statustext_t {
    uint8_t severity; /*< Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.*/
    char text[50]; /*< Status text message, without null termination character*/
};

struct mavlink_command_long_t { /* mavlink/common/COMMAND_LONG */
    uint16_t command; /*<Command ID */
    float param1; /* < parameter 1 */
    float param2; /* < parameter 2 */
    float param3; /* < parameter 3 */
    float param4; /* < parameter 4 */
    float param5; /* < parameter 5 */
    float param6; /* < parameter 6 */
    float param7; /* < parameter 7 */
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
    uint8_t confirmation; /*< Confirmation, 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command) */
};

struct mavlink_command_ack_t { /* mavlink/common/COMMAND_ACK */
    uint16_t command; /*<Command ID (of acknowledged command)*/
    uint8_t result; /*<Result of command */
};

struct mavlink_set_postion_target_local_ned_t { /* mavlink/common/set_position_target_local_ned */
    uint32_t time_boot_ms; /*< Timestamp (time since system boot). */
    float x; /*< X Position in NED frame */
    float y; /*< Y Position in NED frame */
    float z; /*< Z Position in NED frame (note, altitude is negative in NED) */
    float vx; /*< X velocity in NED frame */
    float vy; /*< Y velocity in NED frame */
    float vz; /*< Z velocity in NED frame */
    float afx; /*< X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N */
    float afy; /*< Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N */
    float afz; /*< Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N */
    float yaw; /*< yaw setpoint */
    float yaw_rate; /*< yaw rate setpoint */
    uint16_t type_mask; /*< POSITION_TARGET_TYPEMASK enum. Bitmap to indicate which dimensions should be ignored by the vehicle. */
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
    uint8_t coordinate_frame; /*< MAV_FRAME enum, coordinate frame to use. Valid options: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9 */
};

struct mavlink_radio_status_t { /* mavlink/common/radio_status */
    uint16_t rxerrors; /*< Count of error corrected radio packets (since boot). */
    uint16_t fixed; /*< Count of radio packet receive errors (since boot). */
    uint8_t rssi; /*< Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown. */
    uint8_t remrssi; /*< Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown. */
    uint8_t txbuf; /*< Remaining free transmitter buffer space. [%] */
    uint8_t noise; /*< Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown. */
    uint8_t remnoise; /*< Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown. */
};

struct mavlink_heartbeat_t { /* mavlink/common/heartbeat */
    uint32_t custom_mode; /*< A bitfield for use for autopilot-specific flags */
    uint8_t type; /*< Type of the MAV (quadrotor, helicopter, etc.) */
    uint8_t autopilot; /*< Autopilot type / class. */
    uint8_t base_mode; /*< System mode bitmap. */
    uint8_t system_status; /*< System status flag. */
    uint8_t system_id;
};

struct mavlink_msg_t {
    uint8_t payload_len;
    uint8_t seq;
    uint8_t sys_id;
    uint8_t comp_id;
    uint8_t msg_id;
    std::vector<uint8_t> payload;
    uint16_t checksum;
};

struct mavlink_lora_mission_ack_t {
    int result;
    std::string result_text;
};

struct mavlink_lora_command_ack_t {
    int command;
    int result;
    std::string result_text;
};

struct mavlink_mission_partial_write_list_t {
    int16_t start_index; /*< Start index. Must be smaller / equal to the largest index of the current onboard list. */
    int16_t end_index; /*< End index, equal or greater than start index. */
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
};

struct mavlink_system_time_t {
    uint64_t time_unix_usec; /*< timestamp (UNIX epoch time) */
    uint32_t time_boot_ms; /*< timestamp (time since system boot) */
};

struct mavlink_altitude_t {
    uint64_t time_usec; /*< Timestamp (UNIX Epoch time or time since system boot). */
    float altitude_monotonic; /*< This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change).  */
    float altitude_amsl; /*< This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to.  */
    float altitude_local; /*< This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive. */
    float altitude_relative; /*< his is the altitude above the home position. It resets on each change of the current home position. */
    float altitude_terrain; /*< This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.*/
    float bottom_clearence; /*< This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.*/
};

struct mavlink_battery_status_t {
    int32_t current_consumed; /*< Consumed charge, -1: autopilot does not provide consumption estimate */
    int32_t energy_consumed; /*< Consumed energy, -1: autopilot does not provide energy consumption estimate */
    uint16_t voltages[10]; /*< Battery voltage of cells. Cells above the valid cell count for this battery should have the UINT16_MAX value. */
    int16_t temperature; /*<  Temperature of the battery. INT16_MAX for unknown temperature. */
    int16_t current_battery; /*< Battery current, -1: autopilot does not measure the current */
    uint8_t id; /*<  Battery ID */
    uint8_t battery_function; /*<  Function of the battery */
    uint8_t type; /*<  Type (chemistry) of the battery */
    int8_t battery_remaining; /*< Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery. */
};

struct mavlink_vibration_t {
    uint64_t time_usec; /*< Timestamp (UNIX Epoch time or time since system boot). */
    float vibration_x; /*< Vibration levels on X-axis */
    float vibration_y; /*< Vibration levels on Y-axis */
    float vibration_z; /*< Vibration levels on Z-axis */
    uint32_t clipping_0; /*< first accelerometer clipping count */
    uint32_t clipping_1; /*< second accelerometer clipping count */
    uint32_t clipping_2; /*< third accelerometer clipping count */
};

struct mavlink_extended_sys_status_t {
    uint8_t vtol_state; /*< The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration. */
    uint8_t landed_state; /*< The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown. */
};

struct mavlink_home_position_t {
    int32_t latitude; /*< Latitude (WGS84) */
    int32_t longitude; /*< Lontitude (WGS84) */
    int32_t altitude; /*< Altitude (MSL) positive up */
    float x; /*< Local X position of this position in the local coordinate frame */
    float y; /*< Local Y position of this position in the local coordinate frame */
    float z; /*< Local Z position of this position in the local coordinate frame */
    float q[4]; /*< World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground */
    float approach_x; /*< 	Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone. */
    float approach_y; /*< 	Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone. */
    float approach_z; /*< 	Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone. */
};

#endif //MAVLINK_BRIDGE_MAVLINK_MESSAGES_HPP
