#pragma once
// MESSAGE STATE_IMU PACKING

#define MAVLINK_MSG_ID_STATE_IMU 203


typedef struct __mavlink_state_imu_t {
 uint8_t lsm6_connected; /*<  LSM6 accelerometers and gyros driver connection state*/
 uint8_t lis3mdl_connected; /*<  LIS3MDL magnetometers driver connection state*/
} mavlink_state_imu_t;

#define MAVLINK_MSG_ID_STATE_IMU_LEN 2
#define MAVLINK_MSG_ID_STATE_IMU_MIN_LEN 2
#define MAVLINK_MSG_ID_203_LEN 2
#define MAVLINK_MSG_ID_203_MIN_LEN 2

#define MAVLINK_MSG_ID_STATE_IMU_CRC 210
#define MAVLINK_MSG_ID_203_CRC 210



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STATE_IMU { \
    203, \
    "STATE_IMU", \
    2, \
    {  { "lsm6_connected", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_state_imu_t, lsm6_connected) }, \
         { "lis3mdl_connected", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_state_imu_t, lis3mdl_connected) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STATE_IMU { \
    "STATE_IMU", \
    2, \
    {  { "lsm6_connected", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_state_imu_t, lsm6_connected) }, \
         { "lis3mdl_connected", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_state_imu_t, lis3mdl_connected) }, \
         } \
}
#endif

/**
 * @brief Pack a state_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lsm6_connected  LSM6 accelerometers and gyros driver connection state
 * @param lis3mdl_connected  LIS3MDL magnetometers driver connection state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t lsm6_connected, uint8_t lis3mdl_connected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_IMU_LEN];
    _mav_put_uint8_t(buf, 0, lsm6_connected);
    _mav_put_uint8_t(buf, 1, lis3mdl_connected);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATE_IMU_LEN);
#else
    mavlink_state_imu_t packet;
    packet.lsm6_connected = lsm6_connected;
    packet.lis3mdl_connected = lis3mdl_connected;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATE_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATE_IMU;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STATE_IMU_MIN_LEN, MAVLINK_MSG_ID_STATE_IMU_LEN, MAVLINK_MSG_ID_STATE_IMU_CRC);
}

/**
 * @brief Pack a state_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lsm6_connected  LSM6 accelerometers and gyros driver connection state
 * @param lis3mdl_connected  LIS3MDL magnetometers driver connection state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t lsm6_connected,uint8_t lis3mdl_connected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_IMU_LEN];
    _mav_put_uint8_t(buf, 0, lsm6_connected);
    _mav_put_uint8_t(buf, 1, lis3mdl_connected);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATE_IMU_LEN);
#else
    mavlink_state_imu_t packet;
    packet.lsm6_connected = lsm6_connected;
    packet.lis3mdl_connected = lis3mdl_connected;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATE_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATE_IMU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STATE_IMU_MIN_LEN, MAVLINK_MSG_ID_STATE_IMU_LEN, MAVLINK_MSG_ID_STATE_IMU_CRC);
}

/**
 * @brief Encode a state_imu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param state_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_state_imu_t* state_imu)
{
    return mavlink_msg_state_imu_pack(system_id, component_id, msg, state_imu->lsm6_connected, state_imu->lis3mdl_connected);
}

/**
 * @brief Encode a state_imu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_imu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_state_imu_t* state_imu)
{
    return mavlink_msg_state_imu_pack_chan(system_id, component_id, chan, msg, state_imu->lsm6_connected, state_imu->lis3mdl_connected);
}

/**
 * @brief Send a state_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param lsm6_connected  LSM6 accelerometers and gyros driver connection state
 * @param lis3mdl_connected  LIS3MDL magnetometers driver connection state
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_state_imu_send(mavlink_channel_t chan, uint8_t lsm6_connected, uint8_t lis3mdl_connected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_IMU_LEN];
    _mav_put_uint8_t(buf, 0, lsm6_connected);
    _mav_put_uint8_t(buf, 1, lis3mdl_connected);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_IMU, buf, MAVLINK_MSG_ID_STATE_IMU_MIN_LEN, MAVLINK_MSG_ID_STATE_IMU_LEN, MAVLINK_MSG_ID_STATE_IMU_CRC);
#else
    mavlink_state_imu_t packet;
    packet.lsm6_connected = lsm6_connected;
    packet.lis3mdl_connected = lis3mdl_connected;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_IMU, (const char *)&packet, MAVLINK_MSG_ID_STATE_IMU_MIN_LEN, MAVLINK_MSG_ID_STATE_IMU_LEN, MAVLINK_MSG_ID_STATE_IMU_CRC);
#endif
}

/**
 * @brief Send a state_imu message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_state_imu_send_struct(mavlink_channel_t chan, const mavlink_state_imu_t* state_imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_state_imu_send(chan, state_imu->lsm6_connected, state_imu->lis3mdl_connected);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_IMU, (const char *)state_imu, MAVLINK_MSG_ID_STATE_IMU_MIN_LEN, MAVLINK_MSG_ID_STATE_IMU_LEN, MAVLINK_MSG_ID_STATE_IMU_CRC);
#endif
}

#if MAVLINK_MSG_ID_STATE_IMU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_state_imu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t lsm6_connected, uint8_t lis3mdl_connected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, lsm6_connected);
    _mav_put_uint8_t(buf, 1, lis3mdl_connected);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_IMU, buf, MAVLINK_MSG_ID_STATE_IMU_MIN_LEN, MAVLINK_MSG_ID_STATE_IMU_LEN, MAVLINK_MSG_ID_STATE_IMU_CRC);
#else
    mavlink_state_imu_t *packet = (mavlink_state_imu_t *)msgbuf;
    packet->lsm6_connected = lsm6_connected;
    packet->lis3mdl_connected = lis3mdl_connected;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_IMU, (const char *)packet, MAVLINK_MSG_ID_STATE_IMU_MIN_LEN, MAVLINK_MSG_ID_STATE_IMU_LEN, MAVLINK_MSG_ID_STATE_IMU_CRC);
#endif
}
#endif

#endif

// MESSAGE STATE_IMU UNPACKING


/**
 * @brief Get field lsm6_connected from state_imu message
 *
 * @return  LSM6 accelerometers and gyros driver connection state
 */
static inline uint8_t mavlink_msg_state_imu_get_lsm6_connected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field lis3mdl_connected from state_imu message
 *
 * @return  LIS3MDL magnetometers driver connection state
 */
static inline uint8_t mavlink_msg_state_imu_get_lis3mdl_connected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a state_imu message into a struct
 *
 * @param msg The message to decode
 * @param state_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_state_imu_decode(const mavlink_message_t* msg, mavlink_state_imu_t* state_imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    state_imu->lsm6_connected = mavlink_msg_state_imu_get_lsm6_connected(msg);
    state_imu->lis3mdl_connected = mavlink_msg_state_imu_get_lis3mdl_connected(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STATE_IMU_LEN? msg->len : MAVLINK_MSG_ID_STATE_IMU_LEN;
        memset(state_imu, 0, MAVLINK_MSG_ID_STATE_IMU_LEN);
    memcpy(state_imu, _MAV_PAYLOAD(msg), len);
#endif
}
