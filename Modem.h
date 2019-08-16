//
// Created by jin on 6/24/19.
//

#ifndef WEBSOCKET_TEST_MODEM_H
#define WEBSOCKET_TEST_MODEM_H

#include <string>
#include "mavlink/include/mavlink/v2.0/ardupilotmega/mavlink.h"
#include "mavlink/include/mavlink/v2.0/ardupilotmega/mavlink_msg_modem_rssi.h"

class Modem {
public:
    Modem();
    ~Modem(){}

    int16_t             get_rssi() { return _rssi;};
    int16_t             get_lte_rssi() { return  _lte_rssi;}
    std::string         &get_network_type() { return _network_type;}
    mavlink_message_t   get_modem_data();

private:
    void                _parse_rssi(const std::string &to_parse);
    void                _resolve_network_type(const std::string &type_name, uint8_t &net_type);
    void                _init_telem_to_send(mavlink_modem_rssi_t &telem);
    mavlink_message_t   _process_modem_output(const std::string & rssi);

private:
    int16_t     _rssi;
    int16_t         _lte_rssi;
    std::string _network_type;


};


#endif //WEBSOCKET_TEST_MODEM_H
