#pragma once
// MESSAGE LTE_MODEM PACKING

#define MAVLINK_MSG_ID_LTE_MODEM 395

MAVPACKED(
typedef struct __mavlink_lte_modem_t {
 int16_t lte_rssi; /*<  LTE rssi*/
 int16_t rssi; /*<  rssi*/
 char network_type[16]; /*<  Network type*/
}) mavlink_lte_modem_t;

#define MAVLINK_MSG_ID_LTE_MODEM_LEN 20
#define MAVLINK_MSG_ID_LTE_MODEM_MIN_LEN 20
#define MAVLINK_MSG_ID_395_LEN 20
#define MAVLINK_MSG_ID_395_MIN_LEN 20

#define MAVLINK_MSG_ID_LTE_MODEM_CRC 82
#define MAVLINK_MSG_ID_395_CRC 82

#define MAVLINK_MSG_LTE_MODEM_FIELD_NETWORK_TYPE_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LTE_MODEM { \
    395, \
    "LTE_MODEM", \
    3, \
    {  { "network_type", NULL, MAVLINK_TYPE_CHAR, 16, 4, offsetof(mavlink_lte_modem_t, network_type) }, \
         { "lte_rssi", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_lte_modem_t, lte_rssi) }, \
         { "rssi", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_lte_modem_t, rssi) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LTE_MODEM { \
    "LTE_MODEM", \
    3, \
    {  { "network_type", NULL, MAVLINK_TYPE_CHAR, 16, 4, offsetof(mavlink_lte_modem_t, network_type) }, \
         { "lte_rssi", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_lte_modem_t, lte_rssi) }, \
         { "rssi", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_lte_modem_t, rssi) }, \
         } \
}
#endif

/**
 * @brief Pack a lte_modem message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param network_type  Network type
 * @param lte_rssi  LTE rssi
 * @param rssi  rssi
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lte_modem_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *network_type, int16_t lte_rssi, int16_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LTE_MODEM_LEN];
    _mav_put_int16_t(buf, 0, lte_rssi);
    _mav_put_int16_t(buf, 2, rssi);
    _mav_put_char_array(buf, 4, network_type, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LTE_MODEM_LEN);
#else
    mavlink_lte_modem_t packet;
    packet.lte_rssi = lte_rssi;
    packet.rssi = rssi;
    mav_array_memcpy(packet.network_type, network_type, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LTE_MODEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LTE_MODEM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LTE_MODEM_MIN_LEN, MAVLINK_MSG_ID_LTE_MODEM_LEN, MAVLINK_MSG_ID_LTE_MODEM_CRC);
}

/**
 * @brief Pack a lte_modem message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param network_type  Network type
 * @param lte_rssi  LTE rssi
 * @param rssi  rssi
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lte_modem_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *network_type,int16_t lte_rssi,int16_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LTE_MODEM_LEN];
    _mav_put_int16_t(buf, 0, lte_rssi);
    _mav_put_int16_t(buf, 2, rssi);
    _mav_put_char_array(buf, 4, network_type, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LTE_MODEM_LEN);
#else
    mavlink_lte_modem_t packet;
    packet.lte_rssi = lte_rssi;
    packet.rssi = rssi;
    mav_array_memcpy(packet.network_type, network_type, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LTE_MODEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LTE_MODEM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LTE_MODEM_MIN_LEN, MAVLINK_MSG_ID_LTE_MODEM_LEN, MAVLINK_MSG_ID_LTE_MODEM_CRC);
}

/**
 * @brief Encode a lte_modem struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lte_modem C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lte_modem_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lte_modem_t* lte_modem)
{
    return mavlink_msg_lte_modem_pack(system_id, component_id, msg, lte_modem->network_type, lte_modem->lte_rssi, lte_modem->rssi);
}

/**
 * @brief Encode a lte_modem struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lte_modem C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lte_modem_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lte_modem_t* lte_modem)
{
    return mavlink_msg_lte_modem_pack_chan(system_id, component_id, chan, msg, lte_modem->network_type, lte_modem->lte_rssi, lte_modem->rssi);
}

/**
 * @brief Send a lte_modem message
 * @param chan MAVLink channel to send the message
 *
 * @param network_type  Network type
 * @param lte_rssi  LTE rssi
 * @param rssi  rssi
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lte_modem_send(mavlink_channel_t chan, const char *network_type, int16_t lte_rssi, int16_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LTE_MODEM_LEN];
    _mav_put_int16_t(buf, 0, lte_rssi);
    _mav_put_int16_t(buf, 2, rssi);
    _mav_put_char_array(buf, 4, network_type, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LTE_MODEM, buf, MAVLINK_MSG_ID_LTE_MODEM_MIN_LEN, MAVLINK_MSG_ID_LTE_MODEM_LEN, MAVLINK_MSG_ID_LTE_MODEM_CRC);
#else
    mavlink_lte_modem_t packet;
    packet.lte_rssi = lte_rssi;
    packet.rssi = rssi;
    mav_array_memcpy(packet.network_type, network_type, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LTE_MODEM, (const char *)&packet, MAVLINK_MSG_ID_LTE_MODEM_MIN_LEN, MAVLINK_MSG_ID_LTE_MODEM_LEN, MAVLINK_MSG_ID_LTE_MODEM_CRC);
#endif
}

/**
 * @brief Send a lte_modem message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lte_modem_send_struct(mavlink_channel_t chan, const mavlink_lte_modem_t* lte_modem)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lte_modem_send(chan, lte_modem->network_type, lte_modem->lte_rssi, lte_modem->rssi);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LTE_MODEM, (const char *)lte_modem, MAVLINK_MSG_ID_LTE_MODEM_MIN_LEN, MAVLINK_MSG_ID_LTE_MODEM_LEN, MAVLINK_MSG_ID_LTE_MODEM_CRC);
#endif
}

#if MAVLINK_MSG_ID_LTE_MODEM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lte_modem_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *network_type, int16_t lte_rssi, int16_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, lte_rssi);
    _mav_put_int16_t(buf, 2, rssi);
    _mav_put_char_array(buf, 4, network_type, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LTE_MODEM, buf, MAVLINK_MSG_ID_LTE_MODEM_MIN_LEN, MAVLINK_MSG_ID_LTE_MODEM_LEN, MAVLINK_MSG_ID_LTE_MODEM_CRC);
#else
    mavlink_lte_modem_t *packet = (mavlink_lte_modem_t *)msgbuf;
    packet->lte_rssi = lte_rssi;
    packet->rssi = rssi;
    mav_array_memcpy(packet->network_type, network_type, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LTE_MODEM, (const char *)packet, MAVLINK_MSG_ID_LTE_MODEM_MIN_LEN, MAVLINK_MSG_ID_LTE_MODEM_LEN, MAVLINK_MSG_ID_LTE_MODEM_CRC);
#endif
}
#endif

#endif

// MESSAGE LTE_MODEM UNPACKING


/**
 * @brief Get field network_type from lte_modem message
 *
 * @return  Network type
 */
static inline uint16_t mavlink_msg_lte_modem_get_network_type(const mavlink_message_t* msg, char *network_type)
{
    return _MAV_RETURN_char_array(msg, network_type, 16,  4);
}

/**
 * @brief Get field lte_rssi from lte_modem message
 *
 * @return  LTE rssi
 */
static inline int16_t mavlink_msg_lte_modem_get_lte_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field rssi from lte_modem message
 *
 * @return  rssi
 */
static inline int16_t mavlink_msg_lte_modem_get_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Decode a lte_modem message into a struct
 *
 * @param msg The message to decode
 * @param lte_modem C-struct to decode the message contents into
 */
static inline void mavlink_msg_lte_modem_decode(const mavlink_message_t* msg, mavlink_lte_modem_t* lte_modem)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lte_modem->lte_rssi = mavlink_msg_lte_modem_get_lte_rssi(msg);
    lte_modem->rssi = mavlink_msg_lte_modem_get_rssi(msg);
    mavlink_msg_lte_modem_get_network_type(msg, lte_modem->network_type);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LTE_MODEM_LEN? msg->len : MAVLINK_MSG_ID_LTE_MODEM_LEN;
        memset(lte_modem, 0, MAVLINK_MSG_ID_LTE_MODEM_LEN);
    memcpy(lte_modem, _MAV_PAYLOAD(msg), len);
#endif
}
