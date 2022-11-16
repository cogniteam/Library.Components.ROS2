#pragma once
// MESSAGE STATE_PID PACKING

#define MAVLINK_MSG_ID_STATE_PID 210


typedef struct __mavlink_state_pid_t {
 int32_t set_point; /*<  Target value*/
 int32_t current_value; /*<  Current measurement*/
 int32_t command; /*<  Output command value*/
} mavlink_state_pid_t;

#define MAVLINK_MSG_ID_STATE_PID_LEN 12
#define MAVLINK_MSG_ID_STATE_PID_MIN_LEN 12
#define MAVLINK_MSG_ID_210_LEN 12
#define MAVLINK_MSG_ID_210_MIN_LEN 12

#define MAVLINK_MSG_ID_STATE_PID_CRC 159
#define MAVLINK_MSG_ID_210_CRC 159



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STATE_PID { \
    210, \
    "STATE_PID", \
    3, \
    {  { "set_point", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_state_pid_t, set_point) }, \
         { "current_value", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_state_pid_t, current_value) }, \
         { "command", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_state_pid_t, command) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STATE_PID { \
    "STATE_PID", \
    3, \
    {  { "set_point", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_state_pid_t, set_point) }, \
         { "current_value", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_state_pid_t, current_value) }, \
         { "command", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_state_pid_t, command) }, \
         } \
}
#endif

/**
 * @brief Pack a state_pid message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param set_point  Target value
 * @param current_value  Current measurement
 * @param command  Output command value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_pid_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t set_point, int32_t current_value, int32_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_PID_LEN];
    _mav_put_int32_t(buf, 0, set_point);
    _mav_put_int32_t(buf, 4, current_value);
    _mav_put_int32_t(buf, 8, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATE_PID_LEN);
#else
    mavlink_state_pid_t packet;
    packet.set_point = set_point;
    packet.current_value = current_value;
    packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATE_PID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATE_PID;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STATE_PID_MIN_LEN, MAVLINK_MSG_ID_STATE_PID_LEN, MAVLINK_MSG_ID_STATE_PID_CRC);
}

/**
 * @brief Pack a state_pid message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_point  Target value
 * @param current_value  Current measurement
 * @param command  Output command value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_pid_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t set_point,int32_t current_value,int32_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_PID_LEN];
    _mav_put_int32_t(buf, 0, set_point);
    _mav_put_int32_t(buf, 4, current_value);
    _mav_put_int32_t(buf, 8, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATE_PID_LEN);
#else
    mavlink_state_pid_t packet;
    packet.set_point = set_point;
    packet.current_value = current_value;
    packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATE_PID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATE_PID;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STATE_PID_MIN_LEN, MAVLINK_MSG_ID_STATE_PID_LEN, MAVLINK_MSG_ID_STATE_PID_CRC);
}

/**
 * @brief Encode a state_pid struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param state_pid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_pid_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_state_pid_t* state_pid)
{
    return mavlink_msg_state_pid_pack(system_id, component_id, msg, state_pid->set_point, state_pid->current_value, state_pid->command);
}

/**
 * @brief Encode a state_pid struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state_pid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_pid_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_state_pid_t* state_pid)
{
    return mavlink_msg_state_pid_pack_chan(system_id, component_id, chan, msg, state_pid->set_point, state_pid->current_value, state_pid->command);
}

/**
 * @brief Send a state_pid message
 * @param chan MAVLink channel to send the message
 *
 * @param set_point  Target value
 * @param current_value  Current measurement
 * @param command  Output command value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_state_pid_send(mavlink_channel_t chan, int32_t set_point, int32_t current_value, int32_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_PID_LEN];
    _mav_put_int32_t(buf, 0, set_point);
    _mav_put_int32_t(buf, 4, current_value);
    _mav_put_int32_t(buf, 8, command);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_PID, buf, MAVLINK_MSG_ID_STATE_PID_MIN_LEN, MAVLINK_MSG_ID_STATE_PID_LEN, MAVLINK_MSG_ID_STATE_PID_CRC);
#else
    mavlink_state_pid_t packet;
    packet.set_point = set_point;
    packet.current_value = current_value;
    packet.command = command;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_PID, (const char *)&packet, MAVLINK_MSG_ID_STATE_PID_MIN_LEN, MAVLINK_MSG_ID_STATE_PID_LEN, MAVLINK_MSG_ID_STATE_PID_CRC);
#endif
}

/**
 * @brief Send a state_pid message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_state_pid_send_struct(mavlink_channel_t chan, const mavlink_state_pid_t* state_pid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_state_pid_send(chan, state_pid->set_point, state_pid->current_value, state_pid->command);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_PID, (const char *)state_pid, MAVLINK_MSG_ID_STATE_PID_MIN_LEN, MAVLINK_MSG_ID_STATE_PID_LEN, MAVLINK_MSG_ID_STATE_PID_CRC);
#endif
}

#if MAVLINK_MSG_ID_STATE_PID_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_state_pid_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t set_point, int32_t current_value, int32_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, set_point);
    _mav_put_int32_t(buf, 4, current_value);
    _mav_put_int32_t(buf, 8, command);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_PID, buf, MAVLINK_MSG_ID_STATE_PID_MIN_LEN, MAVLINK_MSG_ID_STATE_PID_LEN, MAVLINK_MSG_ID_STATE_PID_CRC);
#else
    mavlink_state_pid_t *packet = (mavlink_state_pid_t *)msgbuf;
    packet->set_point = set_point;
    packet->current_value = current_value;
    packet->command = command;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_PID, (const char *)packet, MAVLINK_MSG_ID_STATE_PID_MIN_LEN, MAVLINK_MSG_ID_STATE_PID_LEN, MAVLINK_MSG_ID_STATE_PID_CRC);
#endif
}
#endif

#endif

// MESSAGE STATE_PID UNPACKING


/**
 * @brief Get field set_point from state_pid message
 *
 * @return  Target value
 */
static inline int32_t mavlink_msg_state_pid_get_set_point(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field current_value from state_pid message
 *
 * @return  Current measurement
 */
static inline int32_t mavlink_msg_state_pid_get_current_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field command from state_pid message
 *
 * @return  Output command value
 */
static inline int32_t mavlink_msg_state_pid_get_command(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a state_pid message into a struct
 *
 * @param msg The message to decode
 * @param state_pid C-struct to decode the message contents into
 */
static inline void mavlink_msg_state_pid_decode(const mavlink_message_t* msg, mavlink_state_pid_t* state_pid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    state_pid->set_point = mavlink_msg_state_pid_get_set_point(msg);
    state_pid->current_value = mavlink_msg_state_pid_get_current_value(msg);
    state_pid->command = mavlink_msg_state_pid_get_command(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STATE_PID_LEN? msg->len : MAVLINK_MSG_ID_STATE_PID_LEN;
        memset(state_pid, 0, MAVLINK_MSG_ID_STATE_PID_LEN);
    memcpy(state_pid, _MAV_PAYLOAD(msg), len);
#endif
}
