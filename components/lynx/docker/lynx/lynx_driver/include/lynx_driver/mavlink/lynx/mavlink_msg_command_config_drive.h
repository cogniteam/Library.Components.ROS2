#pragma once
// MESSAGE COMMAND_CONFIG_DRIVE PACKING

#define MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE 102


typedef struct __mavlink_command_config_drive_t {
 int32_t tick_to_meter; /*<  Value of each tick in meters*/
 uint16_t throttle_zero_pwm; /*<  Zero velocity PWM value (from RC)*/
 uint16_t steering_zero_pwm; /*<  Zero angle PWM value for steering servo*/
 uint16_t steering_min_pwm; /*<  Minimum allowed PWM value for steering servo*/
 uint16_t steering_max_pwm; /*<  Maximum allowed PWM value for steering servo*/
 uint16_t throttle_deadzone; /*<  Throttle threshold for releasing the brakes*/
 uint16_t front_breaks_on_pwm; /*<  PWM value for front breaks servo (when front breaks activated)*/
 uint16_t front_breaks_off_pwm; /*<  PWM value for front breaks servo (when front breaks deactivated)*/
 uint16_t rear_breaks_on_pwm; /*<  PWM value for rear breaks servo (when rear breaks activated)*/
 uint16_t rear_breaks_off_pwm; /*<  PWM value for rear breaks servo (when rear breaks deactivated)*/
 uint16_t throttle_min_dac_value; /*<  analogWrite() value to set ~0.8V to throttle DAC pin*/
} mavlink_command_config_drive_t;

#define MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN 24
#define MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_MIN_LEN 24
#define MAVLINK_MSG_ID_102_LEN 24
#define MAVLINK_MSG_ID_102_MIN_LEN 24

#define MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_CRC 132
#define MAVLINK_MSG_ID_102_CRC 132



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_COMMAND_CONFIG_DRIVE { \
    102, \
    "COMMAND_CONFIG_DRIVE", \
    11, \
    {  { "throttle_zero_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_command_config_drive_t, throttle_zero_pwm) }, \
         { "steering_zero_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_command_config_drive_t, steering_zero_pwm) }, \
         { "steering_min_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_command_config_drive_t, steering_min_pwm) }, \
         { "steering_max_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_command_config_drive_t, steering_max_pwm) }, \
         { "throttle_deadzone", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_command_config_drive_t, throttle_deadzone) }, \
         { "front_breaks_on_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_command_config_drive_t, front_breaks_on_pwm) }, \
         { "front_breaks_off_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_command_config_drive_t, front_breaks_off_pwm) }, \
         { "rear_breaks_on_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_command_config_drive_t, rear_breaks_on_pwm) }, \
         { "rear_breaks_off_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_command_config_drive_t, rear_breaks_off_pwm) }, \
         { "tick_to_meter", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_command_config_drive_t, tick_to_meter) }, \
         { "throttle_min_dac_value", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_command_config_drive_t, throttle_min_dac_value) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_COMMAND_CONFIG_DRIVE { \
    "COMMAND_CONFIG_DRIVE", \
    11, \
    {  { "throttle_zero_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_command_config_drive_t, throttle_zero_pwm) }, \
         { "steering_zero_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_command_config_drive_t, steering_zero_pwm) }, \
         { "steering_min_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_command_config_drive_t, steering_min_pwm) }, \
         { "steering_max_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_command_config_drive_t, steering_max_pwm) }, \
         { "throttle_deadzone", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_command_config_drive_t, throttle_deadzone) }, \
         { "front_breaks_on_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_command_config_drive_t, front_breaks_on_pwm) }, \
         { "front_breaks_off_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_command_config_drive_t, front_breaks_off_pwm) }, \
         { "rear_breaks_on_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_command_config_drive_t, rear_breaks_on_pwm) }, \
         { "rear_breaks_off_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_command_config_drive_t, rear_breaks_off_pwm) }, \
         { "tick_to_meter", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_command_config_drive_t, tick_to_meter) }, \
         { "throttle_min_dac_value", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_command_config_drive_t, throttle_min_dac_value) }, \
         } \
}
#endif

/**
 * @brief Pack a command_config_drive message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param throttle_zero_pwm  Zero velocity PWM value (from RC)
 * @param steering_zero_pwm  Zero angle PWM value for steering servo
 * @param steering_min_pwm  Minimum allowed PWM value for steering servo
 * @param steering_max_pwm  Maximum allowed PWM value for steering servo
 * @param throttle_deadzone  Throttle threshold for releasing the brakes
 * @param front_breaks_on_pwm  PWM value for front breaks servo (when front breaks activated)
 * @param front_breaks_off_pwm  PWM value for front breaks servo (when front breaks deactivated)
 * @param rear_breaks_on_pwm  PWM value for rear breaks servo (when rear breaks activated)
 * @param rear_breaks_off_pwm  PWM value for rear breaks servo (when rear breaks deactivated)
 * @param tick_to_meter  Value of each tick in meters
 * @param throttle_min_dac_value  analogWrite() value to set ~0.8V to throttle DAC pin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_config_drive_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t throttle_zero_pwm, uint16_t steering_zero_pwm, uint16_t steering_min_pwm, uint16_t steering_max_pwm, uint16_t throttle_deadzone, uint16_t front_breaks_on_pwm, uint16_t front_breaks_off_pwm, uint16_t rear_breaks_on_pwm, uint16_t rear_breaks_off_pwm, int32_t tick_to_meter, uint16_t throttle_min_dac_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN];
    _mav_put_int32_t(buf, 0, tick_to_meter);
    _mav_put_uint16_t(buf, 4, throttle_zero_pwm);
    _mav_put_uint16_t(buf, 6, steering_zero_pwm);
    _mav_put_uint16_t(buf, 8, steering_min_pwm);
    _mav_put_uint16_t(buf, 10, steering_max_pwm);
    _mav_put_uint16_t(buf, 12, throttle_deadzone);
    _mav_put_uint16_t(buf, 14, front_breaks_on_pwm);
    _mav_put_uint16_t(buf, 16, front_breaks_off_pwm);
    _mav_put_uint16_t(buf, 18, rear_breaks_on_pwm);
    _mav_put_uint16_t(buf, 20, rear_breaks_off_pwm);
    _mav_put_uint16_t(buf, 22, throttle_min_dac_value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN);
#else
    mavlink_command_config_drive_t packet;
    packet.tick_to_meter = tick_to_meter;
    packet.throttle_zero_pwm = throttle_zero_pwm;
    packet.steering_zero_pwm = steering_zero_pwm;
    packet.steering_min_pwm = steering_min_pwm;
    packet.steering_max_pwm = steering_max_pwm;
    packet.throttle_deadzone = throttle_deadzone;
    packet.front_breaks_on_pwm = front_breaks_on_pwm;
    packet.front_breaks_off_pwm = front_breaks_off_pwm;
    packet.rear_breaks_on_pwm = rear_breaks_on_pwm;
    packet.rear_breaks_off_pwm = rear_breaks_off_pwm;
    packet.throttle_min_dac_value = throttle_min_dac_value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_MIN_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_CRC);
}

/**
 * @brief Pack a command_config_drive message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param throttle_zero_pwm  Zero velocity PWM value (from RC)
 * @param steering_zero_pwm  Zero angle PWM value for steering servo
 * @param steering_min_pwm  Minimum allowed PWM value for steering servo
 * @param steering_max_pwm  Maximum allowed PWM value for steering servo
 * @param throttle_deadzone  Throttle threshold for releasing the brakes
 * @param front_breaks_on_pwm  PWM value for front breaks servo (when front breaks activated)
 * @param front_breaks_off_pwm  PWM value for front breaks servo (when front breaks deactivated)
 * @param rear_breaks_on_pwm  PWM value for rear breaks servo (when rear breaks activated)
 * @param rear_breaks_off_pwm  PWM value for rear breaks servo (when rear breaks deactivated)
 * @param tick_to_meter  Value of each tick in meters
 * @param throttle_min_dac_value  analogWrite() value to set ~0.8V to throttle DAC pin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_config_drive_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t throttle_zero_pwm,uint16_t steering_zero_pwm,uint16_t steering_min_pwm,uint16_t steering_max_pwm,uint16_t throttle_deadzone,uint16_t front_breaks_on_pwm,uint16_t front_breaks_off_pwm,uint16_t rear_breaks_on_pwm,uint16_t rear_breaks_off_pwm,int32_t tick_to_meter,uint16_t throttle_min_dac_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN];
    _mav_put_int32_t(buf, 0, tick_to_meter);
    _mav_put_uint16_t(buf, 4, throttle_zero_pwm);
    _mav_put_uint16_t(buf, 6, steering_zero_pwm);
    _mav_put_uint16_t(buf, 8, steering_min_pwm);
    _mav_put_uint16_t(buf, 10, steering_max_pwm);
    _mav_put_uint16_t(buf, 12, throttle_deadzone);
    _mav_put_uint16_t(buf, 14, front_breaks_on_pwm);
    _mav_put_uint16_t(buf, 16, front_breaks_off_pwm);
    _mav_put_uint16_t(buf, 18, rear_breaks_on_pwm);
    _mav_put_uint16_t(buf, 20, rear_breaks_off_pwm);
    _mav_put_uint16_t(buf, 22, throttle_min_dac_value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN);
#else
    mavlink_command_config_drive_t packet;
    packet.tick_to_meter = tick_to_meter;
    packet.throttle_zero_pwm = throttle_zero_pwm;
    packet.steering_zero_pwm = steering_zero_pwm;
    packet.steering_min_pwm = steering_min_pwm;
    packet.steering_max_pwm = steering_max_pwm;
    packet.throttle_deadzone = throttle_deadzone;
    packet.front_breaks_on_pwm = front_breaks_on_pwm;
    packet.front_breaks_off_pwm = front_breaks_off_pwm;
    packet.rear_breaks_on_pwm = rear_breaks_on_pwm;
    packet.rear_breaks_off_pwm = rear_breaks_off_pwm;
    packet.throttle_min_dac_value = throttle_min_dac_value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_MIN_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_CRC);
}

/**
 * @brief Encode a command_config_drive struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_config_drive C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_config_drive_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_config_drive_t* command_config_drive)
{
    return mavlink_msg_command_config_drive_pack(system_id, component_id, msg, command_config_drive->throttle_zero_pwm, command_config_drive->steering_zero_pwm, command_config_drive->steering_min_pwm, command_config_drive->steering_max_pwm, command_config_drive->throttle_deadzone, command_config_drive->front_breaks_on_pwm, command_config_drive->front_breaks_off_pwm, command_config_drive->rear_breaks_on_pwm, command_config_drive->rear_breaks_off_pwm, command_config_drive->tick_to_meter, command_config_drive->throttle_min_dac_value);
}

/**
 * @brief Encode a command_config_drive struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_config_drive C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_config_drive_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_command_config_drive_t* command_config_drive)
{
    return mavlink_msg_command_config_drive_pack_chan(system_id, component_id, chan, msg, command_config_drive->throttle_zero_pwm, command_config_drive->steering_zero_pwm, command_config_drive->steering_min_pwm, command_config_drive->steering_max_pwm, command_config_drive->throttle_deadzone, command_config_drive->front_breaks_on_pwm, command_config_drive->front_breaks_off_pwm, command_config_drive->rear_breaks_on_pwm, command_config_drive->rear_breaks_off_pwm, command_config_drive->tick_to_meter, command_config_drive->throttle_min_dac_value);
}

/**
 * @brief Send a command_config_drive message
 * @param chan MAVLink channel to send the message
 *
 * @param throttle_zero_pwm  Zero velocity PWM value (from RC)
 * @param steering_zero_pwm  Zero angle PWM value for steering servo
 * @param steering_min_pwm  Minimum allowed PWM value for steering servo
 * @param steering_max_pwm  Maximum allowed PWM value for steering servo
 * @param throttle_deadzone  Throttle threshold for releasing the brakes
 * @param front_breaks_on_pwm  PWM value for front breaks servo (when front breaks activated)
 * @param front_breaks_off_pwm  PWM value for front breaks servo (when front breaks deactivated)
 * @param rear_breaks_on_pwm  PWM value for rear breaks servo (when rear breaks activated)
 * @param rear_breaks_off_pwm  PWM value for rear breaks servo (when rear breaks deactivated)
 * @param tick_to_meter  Value of each tick in meters
 * @param throttle_min_dac_value  analogWrite() value to set ~0.8V to throttle DAC pin
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_config_drive_send(mavlink_channel_t chan, uint16_t throttle_zero_pwm, uint16_t steering_zero_pwm, uint16_t steering_min_pwm, uint16_t steering_max_pwm, uint16_t throttle_deadzone, uint16_t front_breaks_on_pwm, uint16_t front_breaks_off_pwm, uint16_t rear_breaks_on_pwm, uint16_t rear_breaks_off_pwm, int32_t tick_to_meter, uint16_t throttle_min_dac_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN];
    _mav_put_int32_t(buf, 0, tick_to_meter);
    _mav_put_uint16_t(buf, 4, throttle_zero_pwm);
    _mav_put_uint16_t(buf, 6, steering_zero_pwm);
    _mav_put_uint16_t(buf, 8, steering_min_pwm);
    _mav_put_uint16_t(buf, 10, steering_max_pwm);
    _mav_put_uint16_t(buf, 12, throttle_deadzone);
    _mav_put_uint16_t(buf, 14, front_breaks_on_pwm);
    _mav_put_uint16_t(buf, 16, front_breaks_off_pwm);
    _mav_put_uint16_t(buf, 18, rear_breaks_on_pwm);
    _mav_put_uint16_t(buf, 20, rear_breaks_off_pwm);
    _mav_put_uint16_t(buf, 22, throttle_min_dac_value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE, buf, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_MIN_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_CRC);
#else
    mavlink_command_config_drive_t packet;
    packet.tick_to_meter = tick_to_meter;
    packet.throttle_zero_pwm = throttle_zero_pwm;
    packet.steering_zero_pwm = steering_zero_pwm;
    packet.steering_min_pwm = steering_min_pwm;
    packet.steering_max_pwm = steering_max_pwm;
    packet.throttle_deadzone = throttle_deadzone;
    packet.front_breaks_on_pwm = front_breaks_on_pwm;
    packet.front_breaks_off_pwm = front_breaks_off_pwm;
    packet.rear_breaks_on_pwm = rear_breaks_on_pwm;
    packet.rear_breaks_off_pwm = rear_breaks_off_pwm;
    packet.throttle_min_dac_value = throttle_min_dac_value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE, (const char *)&packet, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_MIN_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_CRC);
#endif
}

/**
 * @brief Send a command_config_drive message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_command_config_drive_send_struct(mavlink_channel_t chan, const mavlink_command_config_drive_t* command_config_drive)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_command_config_drive_send(chan, command_config_drive->throttle_zero_pwm, command_config_drive->steering_zero_pwm, command_config_drive->steering_min_pwm, command_config_drive->steering_max_pwm, command_config_drive->throttle_deadzone, command_config_drive->front_breaks_on_pwm, command_config_drive->front_breaks_off_pwm, command_config_drive->rear_breaks_on_pwm, command_config_drive->rear_breaks_off_pwm, command_config_drive->tick_to_meter, command_config_drive->throttle_min_dac_value);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE, (const char *)command_config_drive, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_MIN_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_CRC);
#endif
}

#if MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_command_config_drive_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t throttle_zero_pwm, uint16_t steering_zero_pwm, uint16_t steering_min_pwm, uint16_t steering_max_pwm, uint16_t throttle_deadzone, uint16_t front_breaks_on_pwm, uint16_t front_breaks_off_pwm, uint16_t rear_breaks_on_pwm, uint16_t rear_breaks_off_pwm, int32_t tick_to_meter, uint16_t throttle_min_dac_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, tick_to_meter);
    _mav_put_uint16_t(buf, 4, throttle_zero_pwm);
    _mav_put_uint16_t(buf, 6, steering_zero_pwm);
    _mav_put_uint16_t(buf, 8, steering_min_pwm);
    _mav_put_uint16_t(buf, 10, steering_max_pwm);
    _mav_put_uint16_t(buf, 12, throttle_deadzone);
    _mav_put_uint16_t(buf, 14, front_breaks_on_pwm);
    _mav_put_uint16_t(buf, 16, front_breaks_off_pwm);
    _mav_put_uint16_t(buf, 18, rear_breaks_on_pwm);
    _mav_put_uint16_t(buf, 20, rear_breaks_off_pwm);
    _mav_put_uint16_t(buf, 22, throttle_min_dac_value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE, buf, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_MIN_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_CRC);
#else
    mavlink_command_config_drive_t *packet = (mavlink_command_config_drive_t *)msgbuf;
    packet->tick_to_meter = tick_to_meter;
    packet->throttle_zero_pwm = throttle_zero_pwm;
    packet->steering_zero_pwm = steering_zero_pwm;
    packet->steering_min_pwm = steering_min_pwm;
    packet->steering_max_pwm = steering_max_pwm;
    packet->throttle_deadzone = throttle_deadzone;
    packet->front_breaks_on_pwm = front_breaks_on_pwm;
    packet->front_breaks_off_pwm = front_breaks_off_pwm;
    packet->rear_breaks_on_pwm = rear_breaks_on_pwm;
    packet->rear_breaks_off_pwm = rear_breaks_off_pwm;
    packet->throttle_min_dac_value = throttle_min_dac_value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE, (const char *)packet, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_MIN_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_CRC);
#endif
}
#endif

#endif

// MESSAGE COMMAND_CONFIG_DRIVE UNPACKING


/**
 * @brief Get field throttle_zero_pwm from command_config_drive message
 *
 * @return  Zero velocity PWM value (from RC)
 */
static inline uint16_t mavlink_msg_command_config_drive_get_throttle_zero_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field steering_zero_pwm from command_config_drive message
 *
 * @return  Zero angle PWM value for steering servo
 */
static inline uint16_t mavlink_msg_command_config_drive_get_steering_zero_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field steering_min_pwm from command_config_drive message
 *
 * @return  Minimum allowed PWM value for steering servo
 */
static inline uint16_t mavlink_msg_command_config_drive_get_steering_min_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field steering_max_pwm from command_config_drive message
 *
 * @return  Maximum allowed PWM value for steering servo
 */
static inline uint16_t mavlink_msg_command_config_drive_get_steering_max_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field throttle_deadzone from command_config_drive message
 *
 * @return  Throttle threshold for releasing the brakes
 */
static inline uint16_t mavlink_msg_command_config_drive_get_throttle_deadzone(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field front_breaks_on_pwm from command_config_drive message
 *
 * @return  PWM value for front breaks servo (when front breaks activated)
 */
static inline uint16_t mavlink_msg_command_config_drive_get_front_breaks_on_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field front_breaks_off_pwm from command_config_drive message
 *
 * @return  PWM value for front breaks servo (when front breaks deactivated)
 */
static inline uint16_t mavlink_msg_command_config_drive_get_front_breaks_off_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field rear_breaks_on_pwm from command_config_drive message
 *
 * @return  PWM value for rear breaks servo (when rear breaks activated)
 */
static inline uint16_t mavlink_msg_command_config_drive_get_rear_breaks_on_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field rear_breaks_off_pwm from command_config_drive message
 *
 * @return  PWM value for rear breaks servo (when rear breaks deactivated)
 */
static inline uint16_t mavlink_msg_command_config_drive_get_rear_breaks_off_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field tick_to_meter from command_config_drive message
 *
 * @return  Value of each tick in meters
 */
static inline int32_t mavlink_msg_command_config_drive_get_tick_to_meter(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field throttle_min_dac_value from command_config_drive message
 *
 * @return  analogWrite() value to set ~0.8V to throttle DAC pin
 */
static inline uint16_t mavlink_msg_command_config_drive_get_throttle_min_dac_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Decode a command_config_drive message into a struct
 *
 * @param msg The message to decode
 * @param command_config_drive C-struct to decode the message contents into
 */
static inline void mavlink_msg_command_config_drive_decode(const mavlink_message_t* msg, mavlink_command_config_drive_t* command_config_drive)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    command_config_drive->tick_to_meter = mavlink_msg_command_config_drive_get_tick_to_meter(msg);
    command_config_drive->throttle_zero_pwm = mavlink_msg_command_config_drive_get_throttle_zero_pwm(msg);
    command_config_drive->steering_zero_pwm = mavlink_msg_command_config_drive_get_steering_zero_pwm(msg);
    command_config_drive->steering_min_pwm = mavlink_msg_command_config_drive_get_steering_min_pwm(msg);
    command_config_drive->steering_max_pwm = mavlink_msg_command_config_drive_get_steering_max_pwm(msg);
    command_config_drive->throttle_deadzone = mavlink_msg_command_config_drive_get_throttle_deadzone(msg);
    command_config_drive->front_breaks_on_pwm = mavlink_msg_command_config_drive_get_front_breaks_on_pwm(msg);
    command_config_drive->front_breaks_off_pwm = mavlink_msg_command_config_drive_get_front_breaks_off_pwm(msg);
    command_config_drive->rear_breaks_on_pwm = mavlink_msg_command_config_drive_get_rear_breaks_on_pwm(msg);
    command_config_drive->rear_breaks_off_pwm = mavlink_msg_command_config_drive_get_rear_breaks_off_pwm(msg);
    command_config_drive->throttle_min_dac_value = mavlink_msg_command_config_drive_get_throttle_min_dac_value(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN? msg->len : MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN;
        memset(command_config_drive, 0, MAVLINK_MSG_ID_COMMAND_CONFIG_DRIVE_LEN);
    memcpy(command_config_drive, _MAV_PAYLOAD(msg), len);
#endif
}
