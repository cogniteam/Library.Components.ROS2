#pragma once
// MESSAGE COMMAND_DRIVE_MOTOR PACKING

#define MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR 100


typedef struct __mavlink_command_drive_motor_t {
 int16_t duty; /*<  Target PWM for DC motor driver*/
} mavlink_command_drive_motor_t;

#define MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN 2
#define MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_MIN_LEN 2
#define MAVLINK_MSG_ID_100_LEN 2
#define MAVLINK_MSG_ID_100_MIN_LEN 2

#define MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_CRC 104
#define MAVLINK_MSG_ID_100_CRC 104



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_COMMAND_DRIVE_MOTOR { \
    100, \
    "COMMAND_DRIVE_MOTOR", \
    1, \
    {  { "duty", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_command_drive_motor_t, duty) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_COMMAND_DRIVE_MOTOR { \
    "COMMAND_DRIVE_MOTOR", \
    1, \
    {  { "duty", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_command_drive_motor_t, duty) }, \
         } \
}
#endif

/**
 * @brief Pack a command_drive_motor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param duty  Target PWM for DC motor driver
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_drive_motor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t duty)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN];
    _mav_put_int16_t(buf, 0, duty);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN);
#else
    mavlink_command_drive_motor_t packet;
    packet.duty = duty;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_CRC);
}

/**
 * @brief Pack a command_drive_motor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param duty  Target PWM for DC motor driver
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_drive_motor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t duty)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN];
    _mav_put_int16_t(buf, 0, duty);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN);
#else
    mavlink_command_drive_motor_t packet;
    packet.duty = duty;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_CRC);
}

/**
 * @brief Encode a command_drive_motor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_drive_motor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_drive_motor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_drive_motor_t* command_drive_motor)
{
    return mavlink_msg_command_drive_motor_pack(system_id, component_id, msg, command_drive_motor->duty);
}

/**
 * @brief Encode a command_drive_motor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_drive_motor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_drive_motor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_command_drive_motor_t* command_drive_motor)
{
    return mavlink_msg_command_drive_motor_pack_chan(system_id, component_id, chan, msg, command_drive_motor->duty);
}

/**
 * @brief Send a command_drive_motor message
 * @param chan MAVLink channel to send the message
 *
 * @param duty  Target PWM for DC motor driver
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_drive_motor_send(mavlink_channel_t chan, int16_t duty)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN];
    _mav_put_int16_t(buf, 0, duty);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR, buf, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_CRC);
#else
    mavlink_command_drive_motor_t packet;
    packet.duty = duty;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR, (const char *)&packet, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_CRC);
#endif
}

/**
 * @brief Send a command_drive_motor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_command_drive_motor_send_struct(mavlink_channel_t chan, const mavlink_command_drive_motor_t* command_drive_motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_command_drive_motor_send(chan, command_drive_motor->duty);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR, (const char *)command_drive_motor, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_command_drive_motor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t duty)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, duty);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR, buf, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_CRC);
#else
    mavlink_command_drive_motor_t *packet = (mavlink_command_drive_motor_t *)msgbuf;
    packet->duty = duty;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR, (const char *)packet, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_CRC);
#endif
}
#endif

#endif

// MESSAGE COMMAND_DRIVE_MOTOR UNPACKING


/**
 * @brief Get field duty from command_drive_motor message
 *
 * @return  Target PWM for DC motor driver
 */
static inline int16_t mavlink_msg_command_drive_motor_get_duty(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Decode a command_drive_motor message into a struct
 *
 * @param msg The message to decode
 * @param command_drive_motor C-struct to decode the message contents into
 */
static inline void mavlink_msg_command_drive_motor_decode(const mavlink_message_t* msg, mavlink_command_drive_motor_t* command_drive_motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    command_drive_motor->duty = mavlink_msg_command_drive_motor_get_duty(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN? msg->len : MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN;
        memset(command_drive_motor, 0, MAVLINK_MSG_ID_COMMAND_DRIVE_MOTOR_LEN);
    memcpy(command_drive_motor, _MAV_PAYLOAD(msg), len);
#endif
}
