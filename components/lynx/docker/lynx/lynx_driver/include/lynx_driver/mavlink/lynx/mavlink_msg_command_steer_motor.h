#pragma once
// MESSAGE COMMAND_STEER_MOTOR PACKING

#define MAVLINK_MSG_ID_COMMAND_STEER_MOTOR 101


typedef struct __mavlink_command_steer_motor_t {
 int16_t target; /*<  Target PWM for steering servo*/
} mavlink_command_steer_motor_t;

#define MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN 2
#define MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_MIN_LEN 2
#define MAVLINK_MSG_ID_101_LEN 2
#define MAVLINK_MSG_ID_101_MIN_LEN 2

#define MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_CRC 168
#define MAVLINK_MSG_ID_101_CRC 168



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_COMMAND_STEER_MOTOR { \
    101, \
    "COMMAND_STEER_MOTOR", \
    1, \
    {  { "target", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_command_steer_motor_t, target) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_COMMAND_STEER_MOTOR { \
    "COMMAND_STEER_MOTOR", \
    1, \
    {  { "target", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_command_steer_motor_t, target) }, \
         } \
}
#endif

/**
 * @brief Pack a command_steer_motor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target  Target PWM for steering servo
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_steer_motor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN];
    _mav_put_int16_t(buf, 0, target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN);
#else
    mavlink_command_steer_motor_t packet;
    packet.target = target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_STEER_MOTOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_CRC);
}

/**
 * @brief Pack a command_steer_motor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target  Target PWM for steering servo
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_steer_motor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN];
    _mav_put_int16_t(buf, 0, target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN);
#else
    mavlink_command_steer_motor_t packet;
    packet.target = target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_STEER_MOTOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_CRC);
}

/**
 * @brief Encode a command_steer_motor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_steer_motor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_steer_motor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_steer_motor_t* command_steer_motor)
{
    return mavlink_msg_command_steer_motor_pack(system_id, component_id, msg, command_steer_motor->target);
}

/**
 * @brief Encode a command_steer_motor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_steer_motor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_steer_motor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_command_steer_motor_t* command_steer_motor)
{
    return mavlink_msg_command_steer_motor_pack_chan(system_id, component_id, chan, msg, command_steer_motor->target);
}

/**
 * @brief Send a command_steer_motor message
 * @param chan MAVLink channel to send the message
 *
 * @param target  Target PWM for steering servo
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_steer_motor_send(mavlink_channel_t chan, int16_t target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN];
    _mav_put_int16_t(buf, 0, target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR, buf, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_CRC);
#else
    mavlink_command_steer_motor_t packet;
    packet.target = target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR, (const char *)&packet, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_CRC);
#endif
}

/**
 * @brief Send a command_steer_motor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_command_steer_motor_send_struct(mavlink_channel_t chan, const mavlink_command_steer_motor_t* command_steer_motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_command_steer_motor_send(chan, command_steer_motor->target);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR, (const char *)command_steer_motor, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_command_steer_motor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR, buf, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_CRC);
#else
    mavlink_command_steer_motor_t *packet = (mavlink_command_steer_motor_t *)msgbuf;
    packet->target = target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR, (const char *)packet, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_MIN_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_CRC);
#endif
}
#endif

#endif

// MESSAGE COMMAND_STEER_MOTOR UNPACKING


/**
 * @brief Get field target from command_steer_motor message
 *
 * @return  Target PWM for steering servo
 */
static inline int16_t mavlink_msg_command_steer_motor_get_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Decode a command_steer_motor message into a struct
 *
 * @param msg The message to decode
 * @param command_steer_motor C-struct to decode the message contents into
 */
static inline void mavlink_msg_command_steer_motor_decode(const mavlink_message_t* msg, mavlink_command_steer_motor_t* command_steer_motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    command_steer_motor->target = mavlink_msg_command_steer_motor_get_target(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN? msg->len : MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN;
        memset(command_steer_motor, 0, MAVLINK_MSG_ID_COMMAND_STEER_MOTOR_LEN);
    memcpy(command_steer_motor, _MAV_PAYLOAD(msg), len);
#endif
}
