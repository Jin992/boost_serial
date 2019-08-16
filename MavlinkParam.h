//
// Created by jin on 6/10/19.
//

#ifndef WEBSOCKET_TEST_MAVLINKPARAM_H
#define WEBSOCKET_TEST_MAVLINKPARAM_H


#include <stdint-gcc.h>
#include <mavlink/include/mavlink/v2.0/common/mavlink.h>

class MavlinkParam {
public:
    MavlinkParam(char *id, float val, uint16_t index, uint16_t count, uint8_t type);
    MavlinkParam(char *id, char *val, uint16_t index, uint16_t count, uint8_t type);
    char                   *get_param_id();
    float                   get_param_value();
    uint16_t                get_param_index();
    uint16_t                get_param_count();
    uint8_t                 get_param_type();
    void                    get_value_pkt(mavlink_param_value_t &pkt);

    void                    set_param_id(char *);
    void                    set_param_value(float);
    void                    set_param_index(uint16_t);
    void                    set_param_count(uint16_t);
    void                    set_param_type(uint8_t);

private:
    mavlink_param_value_t   value_pkt;
    float                   param_value_;
    uint16_t                param_index_;
    uint16_t         param_count_;
    uint8_t                 param_type_;
    char                    param_id_[16];



};


#endif //WEBSOCKET_TEST_MAVLINKPARAM_H
