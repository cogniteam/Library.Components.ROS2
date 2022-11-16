/** @file
 *    @brief MAVLink comm protocol testsuite generated from lynx.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef LYNX_TESTSUITE_H
#define LYNX_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_lynx(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_lynx(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_command_drive_motor(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_command_drive_motor_t packet_in = {
        17235
    };
    mavlink_command_drive_motor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.duty = packet_in.duty;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_drive_motor_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_command_drive_motor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_drive_motor_pack(system_id, component_id, &msg , packet1.duty );
    mavlink_msg_command_drive_motor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_drive_motor_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.duty );
    mavlink_msg_command_drive_motor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_command_drive_motor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_drive_motor_send(MAVLINK_COMM_1 , packet1.duty );
    mavlink_msg_command_drive_motor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_command_steer_motor(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMMAND_STEER_MOTOR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_command_steer_motor_t packet_in = {
        17235
    };
    mavlink_command_steer_motor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target = packet_in.target;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_steer_motor_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_command_steer_motor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_steer_motor_pack(system_id, component_id, &msg , packet1.target );
    mavlink_msg_command_steer_motor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_steer_motor_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target );
    mavlink_msg_command_steer_motor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_command_steer_motor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_steer_motor_send(MAVLINK_COMM_1 , packet1.target );
    mavlink_msg_command_steer_motor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_command_config_drive(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_command_config_drive_t packet_in = {
        963497464,17443,17547,17651,17755,17859,17963,18067,18171,18275,18379
    };
    mavlink_command_config_drive_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.tick_to_meter = packet_in.tick_to_meter;
        packet1.throttle_zero_pwm = packet_in.throttle_zero_pwm;
        packet1.steering_zero_pwm = packet_in.steering_zero_pwm;
        packet1.steering_min_pwm = packet_in.steering_min_pwm;
        packet1.steering_max_pwm = packet_in.steering_max_pwm;
        packet1.throttle_deadzone = packet_in.throttle_deadzone;
        packet1.front_breaks_on_pwm = packet_in.front_breaks_on_pwm;
        packet1.front_breaks_off_pwm = packet_in.front_breaks_off_pwm;
        packet1.rear_breaks_on_pwm = packet_in.rear_breaks_on_pwm;
        packet1.rear_breaks_off_pwm = packet_in.rear_breaks_off_pwm;
        packet1.throttle_min_dac_value = packet_in.throttle_min_dac_value;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_config_drive_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_command_config_drive_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_config_drive_pack(system_id, component_id, &msg , packet1.throttle_zero_pwm , packet1.steering_zero_pwm , packet1.steering_min_pwm , packet1.steering_max_pwm , packet1.throttle_deadzone , packet1.front_breaks_on_pwm , packet1.front_breaks_off_pwm , packet1.rear_breaks_on_pwm , packet1.rear_breaks_off_pwm , packet1.tick_to_meter , packet1.throttle_min_dac_value );
    mavlink_msg_command_config_drive_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_config_drive_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.throttle_zero_pwm , packet1.steering_zero_pwm , packet1.steering_min_pwm , packet1.steering_max_pwm , packet1.throttle_deadzone , packet1.front_breaks_on_pwm , packet1.front_breaks_off_pwm , packet1.rear_breaks_on_pwm , packet1.rear_breaks_off_pwm , packet1.tick_to_meter , packet1.throttle_min_dac_value );
    mavlink_msg_command_config_drive_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_command_config_drive_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_config_drive_send(MAVLINK_COMM_1 , packet1.throttle_zero_pwm , packet1.steering_zero_pwm , packet1.steering_min_pwm , packet1.steering_max_pwm , packet1.throttle_deadzone , packet1.front_breaks_on_pwm , packet1.front_breaks_off_pwm , packet1.rear_breaks_on_pwm , packet1.rear_breaks_off_pwm , packet1.tick_to_meter , packet1.throttle_min_dac_value );
    mavlink_msg_command_config_drive_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_IMU >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_imu_t packet_in = {
        17235,17339,17443,17547,17651,17755,17859,17963,18067
    };
    mavlink_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.gyro_x = packet_in.gyro_x;
        packet1.gyro_y = packet_in.gyro_y;
        packet1.gyro_z = packet_in.gyro_z;
        packet1.accel_x = packet_in.accel_x;
        packet1.accel_y = packet_in.accel_y;
        packet1.accel_z = packet_in.accel_z;
        packet1.mag_x = packet_in.mag_x;
        packet1.mag_y = packet_in.mag_y;
        packet1.mag_z = packet_in.mag_z;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_IMU_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_IMU_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_pack(system_id, component_id, &msg , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.accel_x , packet1.accel_y , packet1.accel_z , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.accel_x , packet1.accel_y , packet1.accel_z , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_send(MAVLINK_COMM_1 , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.accel_x , packet1.accel_y , packet1.accel_z , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_encoder(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ENCODER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_encoder_t packet_in = {
        963497464,963497672
    };
    mavlink_encoder_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ticks = packet_in.ticks;
        packet1.velocity = packet_in.velocity;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ENCODER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ENCODER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_encoder_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_encoder_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_encoder_pack(system_id, component_id, &msg , packet1.ticks , packet1.velocity );
    mavlink_msg_encoder_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_encoder_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ticks , packet1.velocity );
    mavlink_msg_encoder_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_encoder_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_encoder_send(MAVLINK_COMM_1 , packet1.ticks , packet1.velocity );
    mavlink_msg_encoder_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_state_drive(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STATE_DRIVE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_state_drive_t packet_in = {
        17235,17339,17443,17547,17651,17755,17859,17963,18067,187,254,65,132
    };
    mavlink_state_drive_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.throttle_voltage = packet_in.throttle_voltage;
        packet1.front_breaks_pwm = packet_in.front_breaks_pwm;
        packet1.rear_breaks_pwm = packet_in.rear_breaks_pwm;
        packet1.steering_pwm = packet_in.steering_pwm;
        packet1.rc_offboard_pwm = packet_in.rc_offboard_pwm;
        packet1.rc_steering_pwm = packet_in.rc_steering_pwm;
        packet1.rc_throttle_pwm = packet_in.rc_throttle_pwm;
        packet1.rc_lights_pwm = packet_in.rc_lights_pwm;
        packet1.rc_horn_pwm = packet_in.rc_horn_pwm;
        packet1.offboard = packet_in.offboard;
        packet1.lights_state = packet_in.lights_state;
        packet1.horn_state = packet_in.horn_state;
        packet1.rc_connected = packet_in.rc_connected;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STATE_DRIVE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STATE_DRIVE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_state_drive_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_state_drive_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_state_drive_pack(system_id, component_id, &msg , packet1.offboard , packet1.throttle_voltage , packet1.front_breaks_pwm , packet1.rear_breaks_pwm , packet1.steering_pwm , packet1.lights_state , packet1.horn_state , packet1.rc_connected , packet1.rc_offboard_pwm , packet1.rc_steering_pwm , packet1.rc_throttle_pwm , packet1.rc_lights_pwm , packet1.rc_horn_pwm );
    mavlink_msg_state_drive_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_state_drive_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.offboard , packet1.throttle_voltage , packet1.front_breaks_pwm , packet1.rear_breaks_pwm , packet1.steering_pwm , packet1.lights_state , packet1.horn_state , packet1.rc_connected , packet1.rc_offboard_pwm , packet1.rc_steering_pwm , packet1.rc_throttle_pwm , packet1.rc_lights_pwm , packet1.rc_horn_pwm );
    mavlink_msg_state_drive_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_state_drive_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_state_drive_send(MAVLINK_COMM_1 , packet1.offboard , packet1.throttle_voltage , packet1.front_breaks_pwm , packet1.rear_breaks_pwm , packet1.steering_pwm , packet1.lights_state , packet1.horn_state , packet1.rc_connected , packet1.rc_offboard_pwm , packet1.rc_steering_pwm , packet1.rc_throttle_pwm , packet1.rc_lights_pwm , packet1.rc_horn_pwm );
    mavlink_msg_state_drive_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_state_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STATE_IMU >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_state_imu_t packet_in = {
        5,72
    };
    mavlink_state_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lsm6_connected = packet_in.lsm6_connected;
        packet1.lis3mdl_connected = packet_in.lis3mdl_connected;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STATE_IMU_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STATE_IMU_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_state_imu_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_state_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_state_imu_pack(system_id, component_id, &msg , packet1.lsm6_connected , packet1.lis3mdl_connected );
    mavlink_msg_state_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_state_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lsm6_connected , packet1.lis3mdl_connected );
    mavlink_msg_state_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_state_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_state_imu_send(MAVLINK_COMM_1 , packet1.lsm6_connected , packet1.lis3mdl_connected );
    mavlink_msg_state_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_state_pid(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STATE_PID >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_state_pid_t packet_in = {
        963497464,963497672,963497880
    };
    mavlink_state_pid_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.set_point = packet_in.set_point;
        packet1.current_value = packet_in.current_value;
        packet1.command = packet_in.command;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STATE_PID_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STATE_PID_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_state_pid_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_state_pid_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_state_pid_pack(system_id, component_id, &msg , packet1.set_point , packet1.current_value , packet1.command );
    mavlink_msg_state_pid_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_state_pid_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.set_point , packet1.current_value , packet1.command );
    mavlink_msg_state_pid_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_state_pid_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_state_pid_send(MAVLINK_COMM_1 , packet1.set_point , packet1.current_value , packet1.command );
    mavlink_msg_state_pid_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_lynx(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_command_drive_motor(system_id, component_id, last_msg);
    mavlink_test_command_steer_motor(system_id, component_id, last_msg);
    mavlink_test_command_config_drive(system_id, component_id, last_msg);
    mavlink_test_imu(system_id, component_id, last_msg);
    mavlink_test_encoder(system_id, component_id, last_msg);
    mavlink_test_state_drive(system_id, component_id, last_msg);
    mavlink_test_state_imu(system_id, component_id, last_msg);
    mavlink_test_state_pid(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // LYNX_TESTSUITE_H
