#pragma once
// MESSAGE STATE_DRIVE PACKING

#define MAVLINK_MSG_ID_STATE_DRIVE 202


typedef struct __mavlink_state_drive_t {
 int16_t throttle_voltage; /*<  Current throttle voltage output value*/
 int16_t front_breaks_pwm; /*<  Current breaks PWM value*/
 int16_t rear_breaks_pwm; /*<  Current breaks PWM value*/
 int16_t steering_pwm; /*<  Current steering motor PWM value*/
 int16_t rc_offboard_pwm; /*<  RC input offboard channel PWM*/
 int16_t rc_steering_pwm; /*<  RC input steering PWM*/
 int16_t rc_throttle_pwm; /*<  RC input throttle PWM*/
 int16_t rc_lights_pwm; /*<  RC input lights PWM*/
 int16_t rc_horn_pwm; /*<  RC input horn PWM*/
 uint8_t offboard; /*<  Is offboard mode enabled*/
 uint8_t lights_state; /*<  Lights enabled state*/
 uint8_t horn_state; /*<  Horn enabled state*/
 uint8_t rc_connected; /*<  RC connection state*/
} mavlink_state_drive_t;

#define MAVLINK_MSG_ID_STATE_DRIVE_LEN 22
#define MAVLINK_MSG_ID_STATE_DRIVE_MIN_LEN 22
#define MAVLINK_MSG_ID_202_LEN 22
#define MAVLINK_MSG_ID_202_MIN_LEN 22

#define MAVLINK_MSG_ID_STATE_DRIVE_CRC 2
#define MAVLINK_MSG_ID_202_CRC 2



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STATE_DRIVE { \
    202, \
    "STATE_DRIVE", \
    13, \
    {  { "offboard", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_state_drive_t, offboard) }, \
         { "throttle_voltage", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_state_drive_t, throttle_voltage) }, \
         { "front_breaks_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_state_drive_t, front_breaks_pwm) }, \
         { "rear_breaks_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_state_drive_t, rear_breaks_pwm) }, \
         { "steering_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_state_drive_t, steering_pwm) }, \
         { "lights_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_state_drive_t, lights_state) }, \
         { "horn_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_state_drive_t, horn_state) }, \
         { "rc_connected", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_state_drive_t, rc_connected) }, \
         { "rc_offboard_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_state_drive_t, rc_offboard_pwm) }, \
         { "rc_steering_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_state_drive_t, rc_steering_pwm) }, \
         { "rc_throttle_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_state_drive_t, rc_throttle_pwm) }, \
         { "rc_lights_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_state_drive_t, rc_lights_pwm) }, \
         { "rc_horn_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_state_drive_t, rc_horn_pwm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STATE_DRIVE { \
    "STATE_DRIVE", \
    13, \
    {  { "offboard", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_state_drive_t, offboard) }, \
         { "throttle_voltage", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_state_drive_t, throttle_voltage) }, \
         { "front_breaks_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_state_drive_t, front_breaks_pwm) }, \
         { "rear_breaks_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_state_drive_t, rear_breaks_pwm) }, \
         { "steering_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_state_drive_t, steering_pwm) }, \
         { "lights_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_state_drive_t, lights_state) }, \
         { "horn_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_state_drive_t, horn_state) }, \
         { "rc_connected", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_state_drive_t, rc_connected) }, \
         { "rc_offboard_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_state_drive_t, rc_offboard_pwm) }, \
         { "rc_steering_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_state_drive_t, rc_steering_pwm) }, \
         { "rc_throttle_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_state_drive_t, rc_throttle_pwm) }, \
         { "rc_lights_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_state_drive_t, rc_lights_pwm) }, \
         { "rc_horn_pwm", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_state_drive_t, rc_horn_pwm) }, \
         } \
}
#endif

/**
 * @brief Pack a state_drive message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param offboard  Is offboard mode enabled
 * @param throttle_voltage  Current throttle voltage output value
 * @param front_breaks_pwm  Current breaks PWM value
 * @param rear_breaks_pwm  Current breaks PWM value
 * @param steering_pwm  Current steering motor PWM value
 * @param lights_state  Lights enabled state
 * @param horn_state  Horn enabled state
 * @param rc_connected  RC connection state
 * @param rc_offboard_pwm  RC input offboard channel PWM
 * @param rc_steering_pwm  RC input steering PWM
 * @param rc_throttle_pwm  RC input throttle PWM
 * @param rc_lights_pwm  RC input lights PWM
 * @param rc_horn_pwm  RC input horn PWM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_drive_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t offboard, int16_t throttle_voltage, int16_t front_breaks_pwm, int16_t rear_breaks_pwm, int16_t steering_pwm, uint8_t lights_state, uint8_t horn_state, uint8_t rc_connected, int16_t rc_offboard_pwm, int16_t rc_steering_pwm, int16_t rc_throttle_pwm, int16_t rc_lights_pwm, int16_t rc_horn_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_DRIVE_LEN];
    _mav_put_int16_t(buf, 0, throttle_voltage);
    _mav_put_int16_t(buf, 2, front_breaks_pwm);
    _mav_put_int16_t(buf, 4, rear_breaks_pwm);
    _mav_put_int16_t(buf, 6, steering_pwm);
    _mav_put_int16_t(buf, 8, rc_offboard_pwm);
    _mav_put_int16_t(buf, 10, rc_steering_pwm);
    _mav_put_int16_t(buf, 12, rc_throttle_pwm);
    _mav_put_int16_t(buf, 14, rc_lights_pwm);
    _mav_put_int16_t(buf, 16, rc_horn_pwm);
    _mav_put_uint8_t(buf, 18, offboard);
    _mav_put_uint8_t(buf, 19, lights_state);
    _mav_put_uint8_t(buf, 20, horn_state);
    _mav_put_uint8_t(buf, 21, rc_connected);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATE_DRIVE_LEN);
#else
    mavlink_state_drive_t packet;
    packet.throttle_voltage = throttle_voltage;
    packet.front_breaks_pwm = front_breaks_pwm;
    packet.rear_breaks_pwm = rear_breaks_pwm;
    packet.steering_pwm = steering_pwm;
    packet.rc_offboard_pwm = rc_offboard_pwm;
    packet.rc_steering_pwm = rc_steering_pwm;
    packet.rc_throttle_pwm = rc_throttle_pwm;
    packet.rc_lights_pwm = rc_lights_pwm;
    packet.rc_horn_pwm = rc_horn_pwm;
    packet.offboard = offboard;
    packet.lights_state = lights_state;
    packet.horn_state = horn_state;
    packet.rc_connected = rc_connected;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATE_DRIVE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATE_DRIVE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STATE_DRIVE_MIN_LEN, MAVLINK_MSG_ID_STATE_DRIVE_LEN, MAVLINK_MSG_ID_STATE_DRIVE_CRC);
}

/**
 * @brief Pack a state_drive message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param offboard  Is offboard mode enabled
 * @param throttle_voltage  Current throttle voltage output value
 * @param front_breaks_pwm  Current breaks PWM value
 * @param rear_breaks_pwm  Current breaks PWM value
 * @param steering_pwm  Current steering motor PWM value
 * @param lights_state  Lights enabled state
 * @param horn_state  Horn enabled state
 * @param rc_connected  RC connection state
 * @param rc_offboard_pwm  RC input offboard channel PWM
 * @param rc_steering_pwm  RC input steering PWM
 * @param rc_throttle_pwm  RC input throttle PWM
 * @param rc_lights_pwm  RC input lights PWM
 * @param rc_horn_pwm  RC input horn PWM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_drive_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t offboard,int16_t throttle_voltage,int16_t front_breaks_pwm,int16_t rear_breaks_pwm,int16_t steering_pwm,uint8_t lights_state,uint8_t horn_state,uint8_t rc_connected,int16_t rc_offboard_pwm,int16_t rc_steering_pwm,int16_t rc_throttle_pwm,int16_t rc_lights_pwm,int16_t rc_horn_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_DRIVE_LEN];
    _mav_put_int16_t(buf, 0, throttle_voltage);
    _mav_put_int16_t(buf, 2, front_breaks_pwm);
    _mav_put_int16_t(buf, 4, rear_breaks_pwm);
    _mav_put_int16_t(buf, 6, steering_pwm);
    _mav_put_int16_t(buf, 8, rc_offboard_pwm);
    _mav_put_int16_t(buf, 10, rc_steering_pwm);
    _mav_put_int16_t(buf, 12, rc_throttle_pwm);
    _mav_put_int16_t(buf, 14, rc_lights_pwm);
    _mav_put_int16_t(buf, 16, rc_horn_pwm);
    _mav_put_uint8_t(buf, 18, offboard);
    _mav_put_uint8_t(buf, 19, lights_state);
    _mav_put_uint8_t(buf, 20, horn_state);
    _mav_put_uint8_t(buf, 21, rc_connected);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATE_DRIVE_LEN);
#else
    mavlink_state_drive_t packet;
    packet.throttle_voltage = throttle_voltage;
    packet.front_breaks_pwm = front_breaks_pwm;
    packet.rear_breaks_pwm = rear_breaks_pwm;
    packet.steering_pwm = steering_pwm;
    packet.rc_offboard_pwm = rc_offboard_pwm;
    packet.rc_steering_pwm = rc_steering_pwm;
    packet.rc_throttle_pwm = rc_throttle_pwm;
    packet.rc_lights_pwm = rc_lights_pwm;
    packet.rc_horn_pwm = rc_horn_pwm;
    packet.offboard = offboard;
    packet.lights_state = lights_state;
    packet.horn_state = horn_state;
    packet.rc_connected = rc_connected;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATE_DRIVE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATE_DRIVE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STATE_DRIVE_MIN_LEN, MAVLINK_MSG_ID_STATE_DRIVE_LEN, MAVLINK_MSG_ID_STATE_DRIVE_CRC);
}

/**
 * @brief Encode a state_drive struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param state_drive C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_drive_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_state_drive_t* state_drive)
{
    return mavlink_msg_state_drive_pack(system_id, component_id, msg, state_drive->offboard, state_drive->throttle_voltage, state_drive->front_breaks_pwm, state_drive->rear_breaks_pwm, state_drive->steering_pwm, state_drive->lights_state, state_drive->horn_state, state_drive->rc_connected, state_drive->rc_offboard_pwm, state_drive->rc_steering_pwm, state_drive->rc_throttle_pwm, state_drive->rc_lights_pwm, state_drive->rc_horn_pwm);
}

/**
 * @brief Encode a state_drive struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state_drive C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_drive_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_state_drive_t* state_drive)
{
    return mavlink_msg_state_drive_pack_chan(system_id, component_id, chan, msg, state_drive->offboard, state_drive->throttle_voltage, state_drive->front_breaks_pwm, state_drive->rear_breaks_pwm, state_drive->steering_pwm, state_drive->lights_state, state_drive->horn_state, state_drive->rc_connected, state_drive->rc_offboard_pwm, state_drive->rc_steering_pwm, state_drive->rc_throttle_pwm, state_drive->rc_lights_pwm, state_drive->rc_horn_pwm);
}

/**
 * @brief Send a state_drive message
 * @param chan MAVLink channel to send the message
 *
 * @param offboard  Is offboard mode enabled
 * @param throttle_voltage  Current throttle voltage output value
 * @param front_breaks_pwm  Current breaks PWM value
 * @param rear_breaks_pwm  Current breaks PWM value
 * @param steering_pwm  Current steering motor PWM value
 * @param lights_state  Lights enabled state
 * @param horn_state  Horn enabled state
 * @param rc_connected  RC connection state
 * @param rc_offboard_pwm  RC input offboard channel PWM
 * @param rc_steering_pwm  RC input steering PWM
 * @param rc_throttle_pwm  RC input throttle PWM
 * @param rc_lights_pwm  RC input lights PWM
 * @param rc_horn_pwm  RC input horn PWM
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_state_drive_send(mavlink_channel_t chan, uint8_t offboard, int16_t throttle_voltage, int16_t front_breaks_pwm, int16_t rear_breaks_pwm, int16_t steering_pwm, uint8_t lights_state, uint8_t horn_state, uint8_t rc_connected, int16_t rc_offboard_pwm, int16_t rc_steering_pwm, int16_t rc_throttle_pwm, int16_t rc_lights_pwm, int16_t rc_horn_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATE_DRIVE_LEN];
    _mav_put_int16_t(buf, 0, throttle_voltage);
    _mav_put_int16_t(buf, 2, front_breaks_pwm);
    _mav_put_int16_t(buf, 4, rear_breaks_pwm);
    _mav_put_int16_t(buf, 6, steering_pwm);
    _mav_put_int16_t(buf, 8, rc_offboard_pwm);
    _mav_put_int16_t(buf, 10, rc_steering_pwm);
    _mav_put_int16_t(buf, 12, rc_throttle_pwm);
    _mav_put_int16_t(buf, 14, rc_lights_pwm);
    _mav_put_int16_t(buf, 16, rc_horn_pwm);
    _mav_put_uint8_t(buf, 18, offboard);
    _mav_put_uint8_t(buf, 19, lights_state);
    _mav_put_uint8_t(buf, 20, horn_state);
    _mav_put_uint8_t(buf, 21, rc_connected);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_DRIVE, buf, MAVLINK_MSG_ID_STATE_DRIVE_MIN_LEN, MAVLINK_MSG_ID_STATE_DRIVE_LEN, MAVLINK_MSG_ID_STATE_DRIVE_CRC);
#else
    mavlink_state_drive_t packet;
    packet.throttle_voltage = throttle_voltage;
    packet.front_breaks_pwm = front_breaks_pwm;
    packet.rear_breaks_pwm = rear_breaks_pwm;
    packet.steering_pwm = steering_pwm;
    packet.rc_offboard_pwm = rc_offboard_pwm;
    packet.rc_steering_pwm = rc_steering_pwm;
    packet.rc_throttle_pwm = rc_throttle_pwm;
    packet.rc_lights_pwm = rc_lights_pwm;
    packet.rc_horn_pwm = rc_horn_pwm;
    packet.offboard = offboard;
    packet.lights_state = lights_state;
    packet.horn_state = horn_state;
    packet.rc_connected = rc_connected;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_DRIVE, (const char *)&packet, MAVLINK_MSG_ID_STATE_DRIVE_MIN_LEN, MAVLINK_MSG_ID_STATE_DRIVE_LEN, MAVLINK_MSG_ID_STATE_DRIVE_CRC);
#endif
}

/**
 * @brief Send a state_drive message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_state_drive_send_struct(mavlink_channel_t chan, const mavlink_state_drive_t* state_drive)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_state_drive_send(chan, state_drive->offboard, state_drive->throttle_voltage, state_drive->front_breaks_pwm, state_drive->rear_breaks_pwm, state_drive->steering_pwm, state_drive->lights_state, state_drive->horn_state, state_drive->rc_connected, state_drive->rc_offboard_pwm, state_drive->rc_steering_pwm, state_drive->rc_throttle_pwm, state_drive->rc_lights_pwm, state_drive->rc_horn_pwm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_DRIVE, (const char *)state_drive, MAVLINK_MSG_ID_STATE_DRIVE_MIN_LEN, MAVLINK_MSG_ID_STATE_DRIVE_LEN, MAVLINK_MSG_ID_STATE_DRIVE_CRC);
#endif
}

#if MAVLINK_MSG_ID_STATE_DRIVE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_state_drive_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t offboard, int16_t throttle_voltage, int16_t front_breaks_pwm, int16_t rear_breaks_pwm, int16_t steering_pwm, uint8_t lights_state, uint8_t horn_state, uint8_t rc_connected, int16_t rc_offboard_pwm, int16_t rc_steering_pwm, int16_t rc_throttle_pwm, int16_t rc_lights_pwm, int16_t rc_horn_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, throttle_voltage);
    _mav_put_int16_t(buf, 2, front_breaks_pwm);
    _mav_put_int16_t(buf, 4, rear_breaks_pwm);
    _mav_put_int16_t(buf, 6, steering_pwm);
    _mav_put_int16_t(buf, 8, rc_offboard_pwm);
    _mav_put_int16_t(buf, 10, rc_steering_pwm);
    _mav_put_int16_t(buf, 12, rc_throttle_pwm);
    _mav_put_int16_t(buf, 14, rc_lights_pwm);
    _mav_put_int16_t(buf, 16, rc_horn_pwm);
    _mav_put_uint8_t(buf, 18, offboard);
    _mav_put_uint8_t(buf, 19, lights_state);
    _mav_put_uint8_t(buf, 20, horn_state);
    _mav_put_uint8_t(buf, 21, rc_connected);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_DRIVE, buf, MAVLINK_MSG_ID_STATE_DRIVE_MIN_LEN, MAVLINK_MSG_ID_STATE_DRIVE_LEN, MAVLINK_MSG_ID_STATE_DRIVE_CRC);
#else
    mavlink_state_drive_t *packet = (mavlink_state_drive_t *)msgbuf;
    packet->throttle_voltage = throttle_voltage;
    packet->front_breaks_pwm = front_breaks_pwm;
    packet->rear_breaks_pwm = rear_breaks_pwm;
    packet->steering_pwm = steering_pwm;
    packet->rc_offboard_pwm = rc_offboard_pwm;
    packet->rc_steering_pwm = rc_steering_pwm;
    packet->rc_throttle_pwm = rc_throttle_pwm;
    packet->rc_lights_pwm = rc_lights_pwm;
    packet->rc_horn_pwm = rc_horn_pwm;
    packet->offboard = offboard;
    packet->lights_state = lights_state;
    packet->horn_state = horn_state;
    packet->rc_connected = rc_connected;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_DRIVE, (const char *)packet, MAVLINK_MSG_ID_STATE_DRIVE_MIN_LEN, MAVLINK_MSG_ID_STATE_DRIVE_LEN, MAVLINK_MSG_ID_STATE_DRIVE_CRC);
#endif
}
#endif

#endif

// MESSAGE STATE_DRIVE UNPACKING


/**
 * @brief Get field offboard from state_drive message
 *
 * @return  Is offboard mode enabled
 */
static inline uint8_t mavlink_msg_state_drive_get_offboard(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field throttle_voltage from state_drive message
 *
 * @return  Current throttle voltage output value
 */
static inline int16_t mavlink_msg_state_drive_get_throttle_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field front_breaks_pwm from state_drive message
 *
 * @return  Current breaks PWM value
 */
static inline int16_t mavlink_msg_state_drive_get_front_breaks_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field rear_breaks_pwm from state_drive message
 *
 * @return  Current breaks PWM value
 */
static inline int16_t mavlink_msg_state_drive_get_rear_breaks_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field steering_pwm from state_drive message
 *
 * @return  Current steering motor PWM value
 */
static inline int16_t mavlink_msg_state_drive_get_steering_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field lights_state from state_drive message
 *
 * @return  Lights enabled state
 */
static inline uint8_t mavlink_msg_state_drive_get_lights_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field horn_state from state_drive message
 *
 * @return  Horn enabled state
 */
static inline uint8_t mavlink_msg_state_drive_get_horn_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field rc_connected from state_drive message
 *
 * @return  RC connection state
 */
static inline uint8_t mavlink_msg_state_drive_get_rc_connected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field rc_offboard_pwm from state_drive message
 *
 * @return  RC input offboard channel PWM
 */
static inline int16_t mavlink_msg_state_drive_get_rc_offboard_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field rc_steering_pwm from state_drive message
 *
 * @return  RC input steering PWM
 */
static inline int16_t mavlink_msg_state_drive_get_rc_steering_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field rc_throttle_pwm from state_drive message
 *
 * @return  RC input throttle PWM
 */
static inline int16_t mavlink_msg_state_drive_get_rc_throttle_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field rc_lights_pwm from state_drive message
 *
 * @return  RC input lights PWM
 */
static inline int16_t mavlink_msg_state_drive_get_rc_lights_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field rc_horn_pwm from state_drive message
 *
 * @return  RC input horn PWM
 */
static inline int16_t mavlink_msg_state_drive_get_rc_horn_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Decode a state_drive message into a struct
 *
 * @param msg The message to decode
 * @param state_drive C-struct to decode the message contents into
 */
static inline void mavlink_msg_state_drive_decode(const mavlink_message_t* msg, mavlink_state_drive_t* state_drive)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    state_drive->throttle_voltage = mavlink_msg_state_drive_get_throttle_voltage(msg);
    state_drive->front_breaks_pwm = mavlink_msg_state_drive_get_front_breaks_pwm(msg);
    state_drive->rear_breaks_pwm = mavlink_msg_state_drive_get_rear_breaks_pwm(msg);
    state_drive->steering_pwm = mavlink_msg_state_drive_get_steering_pwm(msg);
    state_drive->rc_offboard_pwm = mavlink_msg_state_drive_get_rc_offboard_pwm(msg);
    state_drive->rc_steering_pwm = mavlink_msg_state_drive_get_rc_steering_pwm(msg);
    state_drive->rc_throttle_pwm = mavlink_msg_state_drive_get_rc_throttle_pwm(msg);
    state_drive->rc_lights_pwm = mavlink_msg_state_drive_get_rc_lights_pwm(msg);
    state_drive->rc_horn_pwm = mavlink_msg_state_drive_get_rc_horn_pwm(msg);
    state_drive->offboard = mavlink_msg_state_drive_get_offboard(msg);
    state_drive->lights_state = mavlink_msg_state_drive_get_lights_state(msg);
    state_drive->horn_state = mavlink_msg_state_drive_get_horn_state(msg);
    state_drive->rc_connected = mavlink_msg_state_drive_get_rc_connected(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STATE_DRIVE_LEN? msg->len : MAVLINK_MSG_ID_STATE_DRIVE_LEN;
        memset(state_drive, 0, MAVLINK_MSG_ID_STATE_DRIVE_LEN);
    memcpy(state_drive, _MAV_PAYLOAD(msg), len);
#endif
}
