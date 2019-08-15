/** @file
 *    @brief MAVLink comm protocol testsuite generated from modem.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef MODEM_TESTSUITE_H
#define MODEM_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_modem(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_modem(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_lte_modem(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LTE_MODEM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_lte_modem_t packet_in = {
        17235,17339,"EFGHIJKLMNOPQRS"
    };
    mavlink_lte_modem_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lte_rssi = packet_in.lte_rssi;
        packet1.rssi = packet_in.rssi;
        
        mav_array_memcpy(packet1.network_type, packet_in.network_type, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LTE_MODEM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LTE_MODEM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lte_modem_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_lte_modem_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lte_modem_pack(system_id, component_id, &msg , packet1.network_type , packet1.lte_rssi , packet1.rssi );
    mavlink_msg_lte_modem_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lte_modem_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.network_type , packet1.lte_rssi , packet1.rssi );
    mavlink_msg_lte_modem_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_lte_modem_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_lte_modem_send(MAVLINK_COMM_1 , packet1.network_type , packet1.lte_rssi , packet1.rssi );
    mavlink_msg_lte_modem_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_modem(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_lte_modem(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MODEM_TESTSUITE_H
