//
// Created by jin on 6/24/19.
//

#include <string.h>
#include <iostream>
#include <mavlink/include/mavlink/v2.0/mavlink_types.h>
#include "Modem.h"

Modem::Modem():_rssi(0), _lte_rssi(0) {}

void Modem::_parse_rssi(const std::string &to_parse) {
    char net[10];
    int8_t rssi_text_len = 8;
    int8_t lte_text_len = 12;
    int8_t net_text_len = 16;

    bzero(net, 10);
    if (to_parse.find("\"rssi\":\"") == std::string::npos) {
        _rssi = -128;
        _lte_rssi = -128;
        _network_type = "No connection";
        return;
    }
    if ((to_parse.begin() + rssi_text_len + to_parse.find("\"rssi\":\""))[0] == '"') {
        _rssi = 0;
    } else {
        std::string rssi_s(to_parse.begin() + rssi_text_len + to_parse.find("\"rssi\":\""), to_parse.end());
        _rssi = std::stoi(rssi_s, nullptr, 10) * -1;
    }
    if ((to_parse.begin() + lte_text_len + to_parse.find("\"lte_rsrp\":\""))[0] == '"') {
        _lte_rssi = 0;
    } else {
        std::string lte(to_parse.begin() + lte_text_len + to_parse.find("\"lte_rsrp\":\""), to_parse.end());
        _lte_rssi = std::stoi(lte, nullptr, 10) * -1;
    }
    unsigned i = 0;
    for (auto it = to_parse.begin() + net_text_len + to_parse.find("\"network_type\":\""); *it != '"'; it++) {
        net[i++] = *it;
    }
    _network_type = std::string(net, i);
}

void Modem::_resolve_network_type(const std::string &type_name, uint8_t &net_type) {
    if (type_name == "No connection\"")
        net_type = 0;
    else if (type_name == "LTE")
        net_type = 7;
    else if (type_name == "DC-HSPA+")
        net_type = 6;
    else if (type_name == "UMTS")
        net_type = 4;
    else if (type_name == "HSPA+")
        net_type = 5;
    else if (type_name == "NO_SERVICE")
        net_type = 1;
    else if (type_name == "LIMITED_SERVICE_GSM")
        net_type = 2;
    else if (type_name == "EDGE")
        net_type = 3;
    else
        net_type = 100;
}

void Modem::_init_telem_to_send(mavlink_modem_rssi_t &telem) {
    telem.rssi = -128;
    telem.lte_rssi = -128;
    /// 8 == modem disconnected
    telem.net_type = 8;
}

mavlink_message_t Modem::_process_modem_output(const std::string & rssi) {
    mavlink_message_t telem;
    mavlink_modem_rssi_t mod;
    _init_telem_to_send(mod);

    if (rssi != "<h1>Not Found</h1>The requested URL /goform/goform_get_cmd_proc") {
        _parse_rssi(rssi);
        mod.rssi = get_rssi();
        mod.lte_rssi = get_lte_rssi();
        _resolve_network_type(get_network_type(), mod.net_type);
    }
    mavlink_msg_modem_rssi_encode(2, 1, &telem, &mod);
    return telem;
}

mavlink_message_t Modem::get_modem_data() {
    FILE *fp;
    char var[64];
    std::string rssi_request =	std::string("curl --silent 'http://192.168.0.1/goform/goform_get_cmd_process?cmd=lte_rsrp%2Crssi%2Cnetwork_type&multi_data=1&_=1556189721147' -H"
                                              " 'Accept-Encoding: gzip, t-Language: ru-RU,ru;q=0.9,en-US;q=0.8,en;q=0.7' -H 'User-Agent: Mozilla/5.0 (X11; Linux x86_64)"
                                              " AppleWebKit/537.36 (KHTML, like Gecko) Ubuntu Chromium/73.0.3683.86 Chrome/73.0.3683.86 Safari/537.36' -H 'Accept: application/json,"
                                              " text/javascript, */*; q=0.01' -H 'Referer: http://192.168.0.1/index.html' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --compressed");

    // compare current and previous timestamps if diff > 1000 milliseconds(1 second)
    // show bitrate
    fp = popen(rssi_request.c_str(), "r");
    fgets(var, sizeof(var), fp);
    std::string rssi(var);
    pclose(fp);
    mavlink_message_t res = _process_modem_output(rssi);
    return res;
}
