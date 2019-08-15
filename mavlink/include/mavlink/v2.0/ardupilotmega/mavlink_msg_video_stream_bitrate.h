#pragma once
// MESSAGE VIDEO_STREAM_BITRATE PACKING

#define MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE 96

MAVPACKED(
typedef struct __mavlink_video_stream_bitrate_t {
 float Bitrate; /*<  Video stream bitrate.*/
}) mavlink_video_stream_bitrate_t;

#define MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN 4
#define MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_MIN_LEN 4
#define MAVLINK_MSG_ID_96_LEN 4
#define MAVLINK_MSG_ID_96_MIN_LEN 4

#define MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_CRC 45
#define MAVLINK_MSG_ID_96_CRC 45



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VIDEO_STREAM_BITRATE { \
    96, \
    "VIDEO_STREAM_BITRATE", \
    1, \
    {  { "Bitrate", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_video_stream_bitrate_t, Bitrate) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VIDEO_STREAM_BITRATE { \
    "VIDEO_STREAM_BITRATE", \
    1, \
    {  { "Bitrate", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_video_stream_bitrate_t, Bitrate) }, \
         } \
}
#endif

/**
 * @brief Pack a video_stream_bitrate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Bitrate  Video stream bitrate.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_video_stream_bitrate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float Bitrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN];
    _mav_put_float(buf, 0, Bitrate);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN);
#else
    mavlink_video_stream_bitrate_t packet;
    packet.Bitrate = Bitrate;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_CRC);
}

/**
 * @brief Pack a video_stream_bitrate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Bitrate  Video stream bitrate.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_video_stream_bitrate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float Bitrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN];
    _mav_put_float(buf, 0, Bitrate);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN);
#else
    mavlink_video_stream_bitrate_t packet;
    packet.Bitrate = Bitrate;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_CRC);
}

/**
 * @brief Encode a video_stream_bitrate struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param video_stream_bitrate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_video_stream_bitrate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_video_stream_bitrate_t* video_stream_bitrate)
{
    return mavlink_msg_video_stream_bitrate_pack(system_id, component_id, msg, video_stream_bitrate->Bitrate);
}

/**
 * @brief Encode a video_stream_bitrate struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param video_stream_bitrate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_video_stream_bitrate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_video_stream_bitrate_t* video_stream_bitrate)
{
    return mavlink_msg_video_stream_bitrate_pack_chan(system_id, component_id, chan, msg, video_stream_bitrate->Bitrate);
}

/**
 * @brief Send a video_stream_bitrate message
 * @param chan MAVLink channel to send the message
 *
 * @param Bitrate  Video stream bitrate.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_video_stream_bitrate_send(mavlink_channel_t chan, float Bitrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN];
    _mav_put_float(buf, 0, Bitrate);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE, buf, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_CRC);
#else
    mavlink_video_stream_bitrate_t packet;
    packet.Bitrate = Bitrate;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE, (const char *)&packet, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_CRC);
#endif
}

/**
 * @brief Send a video_stream_bitrate message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_video_stream_bitrate_send_struct(mavlink_channel_t chan, const mavlink_video_stream_bitrate_t* video_stream_bitrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_video_stream_bitrate_send(chan, video_stream_bitrate->Bitrate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE, (const char *)video_stream_bitrate, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_video_stream_bitrate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float Bitrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, Bitrate);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE, buf, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_CRC);
#else
    mavlink_video_stream_bitrate_t *packet = (mavlink_video_stream_bitrate_t *)msgbuf;
    packet->Bitrate = Bitrate;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE, (const char *)packet, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_CRC);
#endif
}
#endif

#endif

// MESSAGE VIDEO_STREAM_BITRATE UNPACKING


/**
 * @brief Get field Bitrate from video_stream_bitrate message
 *
 * @return  Video stream bitrate.
 */
static inline float mavlink_msg_video_stream_bitrate_get_Bitrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a video_stream_bitrate message into a struct
 *
 * @param msg The message to decode
 * @param video_stream_bitrate C-struct to decode the message contents into
 */
static inline void mavlink_msg_video_stream_bitrate_decode(const mavlink_message_t* msg, mavlink_video_stream_bitrate_t* video_stream_bitrate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    video_stream_bitrate->Bitrate = mavlink_msg_video_stream_bitrate_get_Bitrate(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN? msg->len : MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN;
        memset(video_stream_bitrate, 0, MAVLINK_MSG_ID_VIDEO_STREAM_BITRATE_LEN);
    memcpy(video_stream_bitrate, _MAV_PAYLOAD(msg), len);
#endif
}
