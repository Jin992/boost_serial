#include <iostream>
#include <boost/asio.hpp>
#include <queue>
#include <utility>
#include <mutex>
#include <thread>
#include <condition_variable>
#include "mavlink/include/mavlink/v2.0/common/mavlink.h"
#include <boost/array.hpp>
#include <openssl/md5.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#define SERIAL_BUF_SIZE 128


typedef std::pair<uint8_t *, std::size_t>                mav_buf_t;
std::mutex              storage_mutex;
std::condition_variable cond;

namespace {
    uint32_t generate_mavlink_binary(uint8_t **data, mavlink_message_t &msg) {
        uint8_t tmp[512];
        uint32_t data_size = mavlink_msg_to_send_buffer(tmp, &msg);
        *data = new uint8_t[data_size];
        memcpy(data, tmp, data_size);
        return data_size;
    }

    std::string make_magic_key(std::string const &key) {
        char mdString[33];
        std::string to_transform(key + " this is magic");
        unsigned char digest[MD5_DIGEST_LENGTH];

        MD5((unsigned char*)to_transform.c_str(), to_transform.size(), (unsigned char*)&digest);
        for(int i = 0; i < 16; i++)
            sprintf(&mdString[i*2], "%02x", (unsigned int)digest[i]);
        return std::string(mdString);
    }
}


class MavProxy {
public:
    MavProxy(std::string const & dev_name, uint32_t baud_rate):_io(), _serial(_io, dev_name), _socket(_io)
    {
        _serial.set_option( boost::asio::serial_port_base::parity() );	// default none
        _serial.set_option( boost::asio::serial_port_base::character_size( 8 ) );
        _serial.set_option( boost::asio::serial_port_base::stop_bits() );	// default one
        _serial.set_option( boost::asio::serial_port_base::baud_rate( baud_rate ) );
        std::cout << "Serial is on" << std::endl;
        boost::asio::ip::udp::resolver resolver(_io);                                                                           /// initialize resolver
        boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(), "192.168.0.135", "25090");           /// setup protocol + ip + port query for resolver
        _remote_endpoint = *resolver.resolve(query);/// initialize endpoint with address resolver
        _socket = boost::asio::ip::udp::socket(_io);
        _socket.open(boost::asio::ip::udp::v4());
        if (!_socket.is_open()) {
            throw "UdpControlStream: failed to open socket";                                                                       /// temporary exception stub
        }
        std::cout << "Socket is on" << std::endl;
        _magic = make_magic_key("Radion");
        _read_serial();
        _read_udp();
        boost::thread t(boost::bind(&boost::asio::io_service::run, &_io));
        if (t.joinable())
            t.join();
    }

private:


    void _read_udp() {
        boost::asio::ip::udp::endpoint r_ep;
        std::cout << "red udp" << std::endl;
        _socket.async_receive_from(boost::asio::buffer(_buf_in), r_ep,
                [&](const boost::system::error_code &error, uint bytes_transferred){
                                            _read_udp_hdl(error, bytes_transferred);
        });
    }

    void _read_udp_hdl(const boost::system::error_code &error, uint bytes_transferred) {
        if(error)
            std::cout << error.message() << std::endl;
        std::cout << "read udp hdl" << std::endl;
        _serial.async_write_some(boost::asio::buffer(_buf_in.data(),bytes_transferred),
                                 [&](const boost::system::error_code &error, std::size_t len){ _write_serial_hdl(error, len);});
    }


    void _write_serial_hdl(const boost::system::error_code &error, uint bytes_transferred) {
        if (error)
            std::cout << error.message() << std::endl;
        std::cout << "write serial hdl" << std::endl;
        _read_udp();
    }

//-----------------------------------------------------------------------------------------------------------------------------------------
    void _read_serial() {
        std::cout << "read serial " << std::endl;
        _serial.async_read_some(boost::asio::buffer(_buffer, SERIAL_BUF_SIZE),
                                [&](boost::system::error_code ec, std::size_t len){_read_serial_handler(ec, len);});
    }

    void _read_serial_handler(const boost::system::error_code &error, uint bytes_transferred) {
        std::cout << "read serial hdl" << std::endl;
        mavlink_status_t status;
        bool state = true;
        for (std::size_t i = 0; i < bytes_transferred; i++){
            if (mavlink_parse_char(MAVLINK_COMM_0, _buffer[i], &_msg, &status)) {
                uint8_t *data;
                _write_udp();
                state = false;
            }
        }
        if (state)
            _read_serial();
    }

    void _write_udp() {
        std::cout << "write hdl" << std::endl;
        char tmp[512];
        uint32_t data_size = mavlink_msg_to_send_buffer((uint8_t *)tmp, &_msg);
        std::shared_ptr<std::string> send_buf = std::make_shared<std::string>(_magic);
        send_buf->append(tmp, data_size);
        _socket.async_send_to(boost::asio::buffer(send_buf->data(), send_buf->size()), _remote_endpoint,
                              [&](const boost::system::error_code& error, std::size_t bytes_transferred){
                                  _write_udp_hdl(error, bytes_transferred, send_buf);});
    }

    void _write_udp_hdl(const boost::system::error_code& error, std::size_t bytes_transferred, std::shared_ptr<std::string> ptr) {
        if (error)
            std::cout << error.message() << std::endl;
        std::cout << "write udp hdl" << std::endl;
        _read_serial();
    }


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




int main() {
    MavProxy uart("/dev/ttyS0", 115200);
    std::cout << "Finish" << std::endl;
    return 0;
}