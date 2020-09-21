cmake_minimum_required(VERSION 3.7)
project(mavlink_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(
        DIRECTORY msg
        FILES
        mavlink_lora_msg.msg
        mavlink_lora_status.msg
        mavlink_lora_pos.msg
        mavlink_lora_attitude.msg
        mavlink_lora_mission_item_int.msg
        mavlink_lora_mission_list.msg
        mavlink_lora_mission_ack.msg
        mavlink_lora_command_ack.msg
        mavlink_lora_command_start_mission.msg
        mavlink_lora_command_set_mode.msg
        mavlink_lora_statustext.msg
        mavlink_lora_heartbeat.msg
        mavlink_lora_set_position_target_local_ned.msg
        mavlink_lora_enable_offboard.msg
        mavlink_lora_command_takeoff.msg
        mavlink_lora_command_land.msg
        mavlink_lora_command_reposition.msg
        mavlink_lora_gps_raw.msg
        mavlink_lora_radio_status.msg
        mavlink_lora_command_message_interval.msg
        mavlink_lora_system_time.msg
        mavlink_lora_altitude.msg
        mavlink_lora_battery_status.msg
        mavlink_lora_vibration.msg
        mavlink_lora_extended_sys_status.msg
        mavlink_lora_command_long.msg
        mavlink_lora_home_position.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        std_msgs
        message_runtime
)
