//
// Created by jin on 8/16/19.
//

#ifndef MAVLINKBOOST_MAVPROXY_H
#define MAVLINKBOOST_MAVPROXY_H

#include <string>
#include <boost/asio.hpp>
#include <thread>
#include "mavlink/include/mavlink/v2.0/common/mavlink.h"
#include <boost/array.hpp>
#include <openssl/md5.h>


#include <iostream>

#define SERIAL_BUF_SIZE 32768

class MavProxy {
public:
    MavProxy(std::string const & dev_name, uint32_t baud_rate);
    void run();

private:
    void _init_serial(uint32_t baud_rate);
    void _init_udp_socket(const std::string &ip, const std::string &port);
    void _read_udp();
    void _read_udp_hdl(const boost::system::error_code &error, uint bytes_transferred);
    void _write_serial_hdl(const boost::system::error_code &error, uint bytes_transferred);

//-----------------------------------------------------------------------------------------------------------------------------------------
    void _read_serial();
    void _read_serial_handler(const boost::system::error_code &error, uint bytes_transferred);
    void _write_udp(mavlink_message_t msg);
    void _write_udp_hdl(const boost::system::error_code& error, std::size_t bytes_transferred, std::shared_ptr<std::string> ptr);

private:
    boost::asio::io_service         _io;
    boost::asio::serial_port        _serial;
    boost::asio::ip::udp::socket    _socket;
    boost::asio::ip::udp::endpoint  _remote_endpoint;
    char                            _buffer[SERIAL_BUF_SIZE];
    std::string                     _magic;
    boost::array<int8_t, 1024>      _buf_in;
    mavlink_message_t               _msg;
};

#endif //MAVLINKBOOST_MAVPROXY_H
