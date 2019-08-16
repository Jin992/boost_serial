//
// Created by jin on 6/10/19.
//

#include "MavlinkParam.h"
#include <string.h>

MavlinkParam::MavlinkParam(char *id, float val, uint16_t index, uint16_t count, uint8_t type)
: value_pkt{0}, param_value_(val), param_index_(index), param_count_(count), param_type_(type)
{
    if (id == nullptr || strlen(id) > 16)
        throw "MavlinkParam: Bad param id";
    strcpy(param_id_, id);
}

MavlinkParam::MavlinkParam(char *id, char *val, uint16_t index, uint16_t count, uint8_t type)
        : value_pkt{0}, param_index_(index), param_count_(count), param_type_(type)
{
    if (id == nullptr || strlen(id) > 16)
        throw "MavlinkParam: Bad param id";
    if (val == nullptr || strlen(val) > 8)
        throw "MavlinkParam: Bad param value";
    strcpy(param_id_, id);
    size_t val_len = strlen(val);
    memcpy(&param_value_, val, val_len);
}

char* MavlinkParam::get_param_id() {
    return param_id_;
}

float MavlinkParam::get_param_value() {
    return param_value_;
}

uint16_t MavlinkParam::get_param_index() {
    return param_index_;
}

uint16_t MavlinkParam::get_param_count() {
    return param_count_;
}

uint8_t MavlinkParam::get_param_type() {
    return param_type_;
}

void MavlinkParam::get_value_pkt(mavlink_param_value_t &pkt) {

}

void MavlinkParam::set_param_id(char *p_id) {

}

void MavlinkParam::set_param_value(float val) {
    param_value_ = val;
}

void MavlinkParam::set_param_index(uint16_t index) {

}

void MavlinkParam::set_param_count(uint16_t count) {

}

void MavlinkParam::set_param_type(uint8_t type) {

}

