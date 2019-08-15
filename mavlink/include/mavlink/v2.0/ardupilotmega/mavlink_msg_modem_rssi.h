#pragma once
// MESSAGE MODEM_RSSI PACKING

#define MAVLINK_MSG_ID_MODEM_RSSI 95

MAVPACKED(
typedef struct __mavlink_modem_rssi_t {
 int16_t rssi; /*<  modem 3G signal strength.*/
 int16_t lte_rssi; /*<  modem 4G/LTE signal strength.*/
 uint8_t net_type; /*<  modem current network type.*/
}) mavlink_modem_rssi_t;

#define MAVLINK_MSG_ID_MODEM_RSSI_LEN 5
#define MAVLINK_MSG_ID_MODEM_RSSI_MIN_LEN 5
#define MAVLINK_MSG_ID_95_LEN 5
#define MAVLINK_MSG_ID_95_MIN_LEN 5

#define MAVLINK_MSG_ID_MODEM_RSSI_CRC 41
#define MAVLINK_MSG_ID_95_CRC 41



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MODEM_RSSI { \
    95, \
    "MODEM_RSSI", \
    3, \
    {  { "rssi", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_modem_rssi_t, rssi) }, \
         { "lte_rssi", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_modem_rssi_t, lte_rssi) }, \
         { "net_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_modem_rssi_t, net_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MODEM_RSSI { \
    "MODEM_RSSI", \
    3, \
    {  { "rssi", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_modem_rssi_t, rssi) }, \
         { "lte_rssi", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_modem_rssi_t, lte_rssi) }, \
         { "net_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_modem_rssi_t, net_type) }, \
         } \
}
#endif

/**
 * @brief Pack a modem_rssi message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rssi  Modem 3G signal strength.
 * @param lte_rssi  Modem 4G/LTE signal strength.
 * @param net_type  Modem current network type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_modem_rssi_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t rssi, int16_t lte_rssi, uint8_t net_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MODEM_RSSI_LEN];
    _mav_put_int16_t(buf, 0, rssi);
    _mav_put_int16_t(buf, 2, lte_rssi);
    _mav_put_uint8_t(buf, 4, net_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MODEM_RSSI_LEN);
#else
    mavlink_modem_rssi_t packet;
    packet.rssi = rssi;
    packet.lte_rssi = lte_rssi;
    packet.net_type = net_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MODEM_RSSI_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MODEM_RSSI;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MODEM_RSSI_MIN_LEN, MAVLINK_MSG_ID_MODEM_RSSI_LEN, MAVLINK_MSG_ID_MODEM_RSSI_CRC);
}

/**
 * @brief Pack a modem_rssi message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rssi  Modem 3G signal strength.
 * @param lte_rssi  Modem 4G/LTE signal strength.
 * @param net_type  Modem current network type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_modem_rssi_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t rssi,int16_t lte_rssi,uint8_t net_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MODEM_RSSI_LEN];
    _mav_put_int16_t(buf, 0, rssi);
    _mav_put_int16_t(buf, 2, lte_rssi);
    _mav_put_uint8_t(buf, 4, net_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MODEM_RSSI_LEN);
#else
    mavlink_modem_rssi_t packet;
    packet.rssi = rssi;
    packet.lte_rssi = lte_rssi;
    packet.net_type = net_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MODEM_RSSI_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MODEM_RSSI;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MODEM_RSSI_MIN_LEN, MAVLINK_MSG_ID_MODEM_RSSI_LEN, MAVLINK_MSG_ID_MODEM_RSSI_CRC);
}

/**
 * @brief Encode a modem_rssi struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param modem_rssi C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_modem_rssi_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_modem_rssi_t* modem_rssi)
{
    return mavlink_msg_modem_rssi_pack(system_id, component_id, msg, modem_rssi->rssi, modem_rssi->lte_rssi, modem_rssi->net_type);
}

/**
 * @brief Encode a modem_rssi struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param modem_rssi C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_modem_rssi_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_modem_rssi_t* modem_rssi)
{
    return mavlink_msg_modem_rssi_pack_chan(system_id, component_id, chan, msg, modem_rssi->rssi, modem_rssi->lte_rssi, modem_rssi->net_type);
}

/**
 * @brief Send a modem_rssi message
 * @param chan MAVLink channel to send the message
 *
 * @param rssi  Modem 3G signal strength.
 * @param lte_rssi  Modem 4G/LTE signal strength.
 * @param net_type  Modem current network type.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_modem_rssi_send(mavlink_channel_t chan, int16_t rssi, int16_t lte_rssi, uint8_t net_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MODEM_RSSI_LEN];
    _mav_put_int16_t(buf, 0, rssi);
    _mav_put_int16_t(buf, 2, lte_rssi);
    _mav_put_uint8_t(buf, 4, net_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MODEM_RSSI, buf, MAVLINK_MSG_ID_MODEM_RSSI_MIN_LEN, MAVLINK_MSG_ID_MODEM_RSSI_LEN, MAVLINK_MSG_ID_MODEM_RSSI_CRC);
#else
    mavlink_modem_rssi_t packet;
    packet.rssi = rssi;
    packet.lte_rssi = lte_rssi;
    packet.net_type = net_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MODEM_RSSI, (const char *)&packet, MAVLINK_MSG_ID_MODEM_RSSI_MIN_LEN, MAVLINK_MSG_ID_MODEM_RSSI_LEN, MAVLINK_MSG_ID_MODEM_RSSI_CRC);
#endif
}

/**
 * @brief Send a modem_rssi message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_modem_rssi_send_struct(mavlink_channel_t chan, const mavlink_modem_rssi_t* modem_rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_modem_rssi_send(chan, modem_rssi->rssi, modem_rssi->lte_rssi, modem_rssi->net_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MODEM_RSSI, (const char *)modem_rssi, MAVLINK_MSG_ID_MODEM_RSSI_MIN_LEN, MAVLINK_MSG_ID_MODEM_RSSI_LEN, MAVLINK_MSG_ID_MODEM_RSSI_CRC);
#endif
}

#if MAVLINK_MSG_ID_MODEM_RSSI_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_modem_rssi_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t rssi, int16_t lte_rssi, uint8_t net_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, rssi);
    _mav_put_int16_t(buf, 2, lte_rssi);
    _mav_put_uint8_t(buf, 4, net_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MODEM_RSSI, buf, MAVLINK_MSG_ID_MODEM_RSSI_MIN_LEN, MAVLINK_MSG_ID_MODEM_RSSI_LEN, MAVLINK_MSG_ID_MODEM_RSSI_CRC);
#else
    mavlink_modem_rssi_t *packet = (mavlink_modem_rssi_t *)msgbuf;
    packet->rssi = rssi;
    packet->lte_rssi = lte_rssi;
    packet->net_type = net_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MODEM_RSSI, (const char *)packet, MAVLINK_MSG_ID_MODEM_RSSI_MIN_LEN, MAVLINK_MSG_ID_MODEM_RSSI_LEN, MAVLINK_MSG_ID_MODEM_RSSI_CRC);
#endif
}
#endif

#endif

// MESSAGE MODEM_RSSI UNPACKING


/**
 * @brief Get field rssi from modem_rssi message
 *
 * @return  Modem 3G signal strength.
 */
static inline int16_t mavlink_msg_modem_rssi_get_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field lte_rssi from modem_rssi message
 *
 * @return  Modem 4G/LTE signal strength.
 */
static inline int16_t mavlink_msg_modem_rssi_get_lte_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field net_type from modem_rssi message
 *
 * @return  Modem current network type.
 */
static inline uint8_t mavlink_msg_modem_rssi_get_net_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a modem_rssi message into a struct
 *
 * @param msg The message to decode
 * @param modem_rssi C-struct to decode the message contents into
 */
static inline void mavlink_msg_modem_rssi_decode(const mavlink_message_t* msg, mavlink_modem_rssi_t* modem_rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    modem_rssi->rssi = mavlink_msg_modem_rssi_get_rssi(msg);
    modem_rssi->lte_rssi = mavlink_msg_modem_rssi_get_lte_rssi(msg);
    modem_rssi->net_type = mavlink_msg_modem_rssi_get_net_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MODEM_RSSI_LEN? msg->len : MAVLINK_MSG_ID_MODEM_RSSI_LEN;
        memset(modem_rssi, 0, MAVLINK_MSG_ID_MODEM_RSSI_LEN);
    memcpy(modem_rssi, _MAV_PAYLOAD(msg), len);
#endif
}
