#pragma once
// MESSAGE ENCODER PACKING

#define MAVLINK_MSG_ID_ENCODER 201


typedef struct __mavlink_encoder_t {
 int32_t ticks; /*<  Total ticks count*/
 int32_t velocity; /*<  Velocity mm/s*/
} mavlink_encoder_t;

#define MAVLINK_MSG_ID_ENCODER_LEN 8
#define MAVLINK_MSG_ID_ENCODER_MIN_LEN 8
#define MAVLINK_MSG_ID_201_LEN 8
#define MAVLINK_MSG_ID_201_MIN_LEN 8

#define MAVLINK_MSG_ID_ENCODER_CRC 136
#define MAVLINK_MSG_ID_201_CRC 136



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ENCODER { \
    201, \
    "ENCODER", \
    2, \
    {  { "ticks", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_encoder_t, ticks) }, \
         { "velocity", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_encoder_t, velocity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ENCODER { \
    "ENCODER", \
    2, \
    {  { "ticks", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_encoder_t, ticks) }, \
         { "velocity", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_encoder_t, velocity) }, \
         } \
}
#endif

/**
 * @brief Pack a encoder message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ticks  Total ticks count
 * @param velocity  Velocity mm/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_encoder_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t ticks, int32_t velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ENCODER_LEN];
    _mav_put_int32_t(buf, 0, ticks);
    _mav_put_int32_t(buf, 4, velocity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ENCODER_LEN);
#else
    mavlink_encoder_t packet;
    packet.ticks = ticks;
    packet.velocity = velocity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ENCODER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ENCODER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ENCODER_MIN_LEN, MAVLINK_MSG_ID_ENCODER_LEN, MAVLINK_MSG_ID_ENCODER_CRC);
}

/**
 * @brief Pack a encoder message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ticks  Total ticks count
 * @param velocity  Velocity mm/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_encoder_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t ticks,int32_t velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ENCODER_LEN];
    _mav_put_int32_t(buf, 0, ticks);
    _mav_put_int32_t(buf, 4, velocity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ENCODER_LEN);
#else
    mavlink_encoder_t packet;
    packet.ticks = ticks;
    packet.velocity = velocity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ENCODER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ENCODER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ENCODER_MIN_LEN, MAVLINK_MSG_ID_ENCODER_LEN, MAVLINK_MSG_ID_ENCODER_CRC);
}

/**
 * @brief Encode a encoder struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param encoder C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_encoder_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_encoder_t* encoder)
{
    return mavlink_msg_encoder_pack(system_id, component_id, msg, encoder->ticks, encoder->velocity);
}

/**
 * @brief Encode a encoder struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param encoder C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_encoder_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_encoder_t* encoder)
{
    return mavlink_msg_encoder_pack_chan(system_id, component_id, chan, msg, encoder->ticks, encoder->velocity);
}

/**
 * @brief Send a encoder message
 * @param chan MAVLink channel to send the message
 *
 * @param ticks  Total ticks count
 * @param velocity  Velocity mm/s
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_encoder_send(mavlink_channel_t chan, int32_t ticks, int32_t velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ENCODER_LEN];
    _mav_put_int32_t(buf, 0, ticks);
    _mav_put_int32_t(buf, 4, velocity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER, buf, MAVLINK_MSG_ID_ENCODER_MIN_LEN, MAVLINK_MSG_ID_ENCODER_LEN, MAVLINK_MSG_ID_ENCODER_CRC);
#else
    mavlink_encoder_t packet;
    packet.ticks = ticks;
    packet.velocity = velocity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER, (const char *)&packet, MAVLINK_MSG_ID_ENCODER_MIN_LEN, MAVLINK_MSG_ID_ENCODER_LEN, MAVLINK_MSG_ID_ENCODER_CRC);
#endif
}

/**
 * @brief Send a encoder message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_encoder_send_struct(mavlink_channel_t chan, const mavlink_encoder_t* encoder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_encoder_send(chan, encoder->ticks, encoder->velocity);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER, (const char *)encoder, MAVLINK_MSG_ID_ENCODER_MIN_LEN, MAVLINK_MSG_ID_ENCODER_LEN, MAVLINK_MSG_ID_ENCODER_CRC);
#endif
}

#if MAVLINK_MSG_ID_ENCODER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_encoder_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t ticks, int32_t velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, ticks);
    _mav_put_int32_t(buf, 4, velocity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER, buf, MAVLINK_MSG_ID_ENCODER_MIN_LEN, MAVLINK_MSG_ID_ENCODER_LEN, MAVLINK_MSG_ID_ENCODER_CRC);
#else
    mavlink_encoder_t *packet = (mavlink_encoder_t *)msgbuf;
    packet->ticks = ticks;
    packet->velocity = velocity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER, (const char *)packet, MAVLINK_MSG_ID_ENCODER_MIN_LEN, MAVLINK_MSG_ID_ENCODER_LEN, MAVLINK_MSG_ID_ENCODER_CRC);
#endif
}
#endif

#endif

// MESSAGE ENCODER UNPACKING


/**
 * @brief Get field ticks from encoder message
 *
 * @return  Total ticks count
 */
static inline int32_t mavlink_msg_encoder_get_ticks(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field velocity from encoder message
 *
 * @return  Velocity mm/s
 */
static inline int32_t mavlink_msg_encoder_get_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Decode a encoder message into a struct
 *
 * @param msg The message to decode
 * @param encoder C-struct to decode the message contents into
 */
static inline void mavlink_msg_encoder_decode(const mavlink_message_t* msg, mavlink_encoder_t* encoder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    encoder->ticks = mavlink_msg_encoder_get_ticks(msg);
    encoder->velocity = mavlink_msg_encoder_get_velocity(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ENCODER_LEN? msg->len : MAVLINK_MSG_ID_ENCODER_LEN;
        memset(encoder, 0, MAVLINK_MSG_ID_ENCODER_LEN);
    memcpy(encoder, _MAV_PAYLOAD(msg), len);
#endif
}
