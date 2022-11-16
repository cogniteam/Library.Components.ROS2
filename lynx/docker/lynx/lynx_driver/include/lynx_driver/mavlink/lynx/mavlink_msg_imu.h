#pragma once
// MESSAGE IMU PACKING

#define MAVLINK_MSG_ID_IMU 200


typedef struct __mavlink_imu_t {
 int16_t gyro_x; /*<  Raw gyro x reading*/
 int16_t gyro_y; /*<  Raw gyro y reading*/
 int16_t gyro_z; /*<  Raw gyro z reading*/
 int16_t accel_x; /*<  Raw accelerometer x reading*/
 int16_t accel_y; /*<  Raw accelerometer y reading*/
 int16_t accel_z; /*<  Raw accelerometer z reading*/
 int16_t mag_x; /*<  Raw magnetometer x reading*/
 int16_t mag_y; /*<  Raw magnetometer y reading*/
 int16_t mag_z; /*<  Raw magnetometer z reading*/
} mavlink_imu_t;

#define MAVLINK_MSG_ID_IMU_LEN 18
#define MAVLINK_MSG_ID_IMU_MIN_LEN 18
#define MAVLINK_MSG_ID_200_LEN 18
#define MAVLINK_MSG_ID_200_MIN_LEN 18

#define MAVLINK_MSG_ID_IMU_CRC 43
#define MAVLINK_MSG_ID_200_CRC 43



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IMU { \
    200, \
    "IMU", \
    9, \
    {  { "gyro_x", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_imu_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_imu_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_imu_t, gyro_z) }, \
         { "accel_x", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_imu_t, accel_x) }, \
         { "accel_y", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_imu_t, accel_y) }, \
         { "accel_z", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_imu_t, accel_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_imu_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_imu_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_imu_t, mag_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IMU { \
    "IMU", \
    9, \
    {  { "gyro_x", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_imu_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_imu_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_imu_t, gyro_z) }, \
         { "accel_x", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_imu_t, accel_x) }, \
         { "accel_y", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_imu_t, accel_y) }, \
         { "accel_z", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_imu_t, accel_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_imu_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_imu_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_imu_t, mag_z) }, \
         } \
}
#endif

/**
 * @brief Pack a imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gyro_x  Raw gyro x reading
 * @param gyro_y  Raw gyro y reading
 * @param gyro_z  Raw gyro z reading
 * @param accel_x  Raw accelerometer x reading
 * @param accel_y  Raw accelerometer y reading
 * @param accel_z  Raw accelerometer z reading
 * @param mag_x  Raw magnetometer x reading
 * @param mag_y  Raw magnetometer y reading
 * @param mag_z  Raw magnetometer z reading
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_LEN];
    _mav_put_int16_t(buf, 0, gyro_x);
    _mav_put_int16_t(buf, 2, gyro_y);
    _mav_put_int16_t(buf, 4, gyro_z);
    _mav_put_int16_t(buf, 6, accel_x);
    _mav_put_int16_t(buf, 8, accel_y);
    _mav_put_int16_t(buf, 10, accel_z);
    _mav_put_int16_t(buf, 12, mag_x);
    _mav_put_int16_t(buf, 14, mag_y);
    _mav_put_int16_t(buf, 16, mag_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_LEN);
#else
    mavlink_imu_t packet;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.accel_x = accel_x;
    packet.accel_y = accel_y;
    packet.accel_z = accel_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
}

/**
 * @brief Pack a imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gyro_x  Raw gyro x reading
 * @param gyro_y  Raw gyro y reading
 * @param gyro_z  Raw gyro z reading
 * @param accel_x  Raw accelerometer x reading
 * @param accel_y  Raw accelerometer y reading
 * @param accel_z  Raw accelerometer z reading
 * @param mag_x  Raw magnetometer x reading
 * @param mag_y  Raw magnetometer y reading
 * @param mag_z  Raw magnetometer z reading
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t gyro_x,int16_t gyro_y,int16_t gyro_z,int16_t accel_x,int16_t accel_y,int16_t accel_z,int16_t mag_x,int16_t mag_y,int16_t mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_LEN];
    _mav_put_int16_t(buf, 0, gyro_x);
    _mav_put_int16_t(buf, 2, gyro_y);
    _mav_put_int16_t(buf, 4, gyro_z);
    _mav_put_int16_t(buf, 6, accel_x);
    _mav_put_int16_t(buf, 8, accel_y);
    _mav_put_int16_t(buf, 10, accel_z);
    _mav_put_int16_t(buf, 12, mag_x);
    _mav_put_int16_t(buf, 14, mag_y);
    _mav_put_int16_t(buf, 16, mag_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_LEN);
#else
    mavlink_imu_t packet;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.accel_x = accel_x;
    packet.accel_y = accel_y;
    packet.accel_z = accel_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
}

/**
 * @brief Encode a imu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_imu_t* imu)
{
    return mavlink_msg_imu_pack(system_id, component_id, msg, imu->gyro_x, imu->gyro_y, imu->gyro_z, imu->accel_x, imu->accel_y, imu->accel_z, imu->mag_x, imu->mag_y, imu->mag_z);
}

/**
 * @brief Encode a imu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_imu_t* imu)
{
    return mavlink_msg_imu_pack_chan(system_id, component_id, chan, msg, imu->gyro_x, imu->gyro_y, imu->gyro_z, imu->accel_x, imu->accel_y, imu->accel_z, imu->mag_x, imu->mag_y, imu->mag_z);
}

/**
 * @brief Send a imu message
 * @param chan MAVLink channel to send the message
 *
 * @param gyro_x  Raw gyro x reading
 * @param gyro_y  Raw gyro y reading
 * @param gyro_z  Raw gyro z reading
 * @param accel_x  Raw accelerometer x reading
 * @param accel_y  Raw accelerometer y reading
 * @param accel_z  Raw accelerometer z reading
 * @param mag_x  Raw magnetometer x reading
 * @param mag_y  Raw magnetometer y reading
 * @param mag_z  Raw magnetometer z reading
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_imu_send(mavlink_channel_t chan, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_LEN];
    _mav_put_int16_t(buf, 0, gyro_x);
    _mav_put_int16_t(buf, 2, gyro_y);
    _mav_put_int16_t(buf, 4, gyro_z);
    _mav_put_int16_t(buf, 6, accel_x);
    _mav_put_int16_t(buf, 8, accel_y);
    _mav_put_int16_t(buf, 10, accel_z);
    _mav_put_int16_t(buf, 12, mag_x);
    _mav_put_int16_t(buf, 14, mag_y);
    _mav_put_int16_t(buf, 16, mag_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, buf, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#else
    mavlink_imu_t packet;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.accel_x = accel_x;
    packet.accel_y = accel_y;
    packet.accel_z = accel_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, (const char *)&packet, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#endif
}

/**
 * @brief Send a imu message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_imu_send_struct(mavlink_channel_t chan, const mavlink_imu_t* imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_imu_send(chan, imu->gyro_x, imu->gyro_y, imu->gyro_z, imu->accel_x, imu->accel_y, imu->accel_z, imu->mag_x, imu->mag_y, imu->mag_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, (const char *)imu, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#endif
}

#if MAVLINK_MSG_ID_IMU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_imu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, gyro_x);
    _mav_put_int16_t(buf, 2, gyro_y);
    _mav_put_int16_t(buf, 4, gyro_z);
    _mav_put_int16_t(buf, 6, accel_x);
    _mav_put_int16_t(buf, 8, accel_y);
    _mav_put_int16_t(buf, 10, accel_z);
    _mav_put_int16_t(buf, 12, mag_x);
    _mav_put_int16_t(buf, 14, mag_y);
    _mav_put_int16_t(buf, 16, mag_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, buf, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#else
    mavlink_imu_t *packet = (mavlink_imu_t *)msgbuf;
    packet->gyro_x = gyro_x;
    packet->gyro_y = gyro_y;
    packet->gyro_z = gyro_z;
    packet->accel_x = accel_x;
    packet->accel_y = accel_y;
    packet->accel_z = accel_z;
    packet->mag_x = mag_x;
    packet->mag_y = mag_y;
    packet->mag_z = mag_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, (const char *)packet, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#endif
}
#endif

#endif

// MESSAGE IMU UNPACKING


/**
 * @brief Get field gyro_x from imu message
 *
 * @return  Raw gyro x reading
 */
static inline int16_t mavlink_msg_imu_get_gyro_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field gyro_y from imu message
 *
 * @return  Raw gyro y reading
 */
static inline int16_t mavlink_msg_imu_get_gyro_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field gyro_z from imu message
 *
 * @return  Raw gyro z reading
 */
static inline int16_t mavlink_msg_imu_get_gyro_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field accel_x from imu message
 *
 * @return  Raw accelerometer x reading
 */
static inline int16_t mavlink_msg_imu_get_accel_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field accel_y from imu message
 *
 * @return  Raw accelerometer y reading
 */
static inline int16_t mavlink_msg_imu_get_accel_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field accel_z from imu message
 *
 * @return  Raw accelerometer z reading
 */
static inline int16_t mavlink_msg_imu_get_accel_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field mag_x from imu message
 *
 * @return  Raw magnetometer x reading
 */
static inline int16_t mavlink_msg_imu_get_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field mag_y from imu message
 *
 * @return  Raw magnetometer y reading
 */
static inline int16_t mavlink_msg_imu_get_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field mag_z from imu message
 *
 * @return  Raw magnetometer z reading
 */
static inline int16_t mavlink_msg_imu_get_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Decode a imu message into a struct
 *
 * @param msg The message to decode
 * @param imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_imu_decode(const mavlink_message_t* msg, mavlink_imu_t* imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    imu->gyro_x = mavlink_msg_imu_get_gyro_x(msg);
    imu->gyro_y = mavlink_msg_imu_get_gyro_y(msg);
    imu->gyro_z = mavlink_msg_imu_get_gyro_z(msg);
    imu->accel_x = mavlink_msg_imu_get_accel_x(msg);
    imu->accel_y = mavlink_msg_imu_get_accel_y(msg);
    imu->accel_z = mavlink_msg_imu_get_accel_z(msg);
    imu->mag_x = mavlink_msg_imu_get_mag_x(msg);
    imu->mag_y = mavlink_msg_imu_get_mag_y(msg);
    imu->mag_z = mavlink_msg_imu_get_mag_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IMU_LEN? msg->len : MAVLINK_MSG_ID_IMU_LEN;
        memset(imu, 0, MAVLINK_MSG_ID_IMU_LEN);
    memcpy(imu, _MAV_PAYLOAD(msg), len);
#endif
}
