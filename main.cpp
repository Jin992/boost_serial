#include <iostream>
#include <boost/asio.hpp>
#include <queue>
#include <utility>
#include <mutex>
#include <thread>
#include <condition_variable>
#include "mavlink/include/mavlink/v2.0/common/mavlink.h"
#define SERIAL_BUF_SIZE 12

typedef std::queue<std::pair<int8_t *, std::size_t>> mav_queue_t;

mav_queue_t             storage;
std::mutex              storage_mutex;
std::condition_variable cond;


class Serial {
public:
    Serial(std::string const & dev_name, uint32_t baud_rate):_io(), _serial(_io, dev_name)
    {
      _serial.set_option( boost::asio::serial_port_base::parity() );	// default none
      _serial.set_option( boost::asio::serial_port_base::character_size( 8 ) );
      _serial.set_option( boost::asio::serial_port_base::stop_bits() );	// default one
      _serial.set_option( boost::asio::serial_port_base::baud_rate( baud_rate ) );

      read_some();
      _io.run();
    }
private:
    void check_buffer() {

    }

    void _handler(const boost::system::error_code &error, uint bytes_transferred) {
        bool success = false;
        for (std::size_t i = 0; i < bytes_transferred; i++){
            mavlink_message_t msg;
            mavlink_status_t status;
            success = mavlink_parse_char(MAVLINK_COMM_0, _buffer[i], &msg, &status);
            if (success)
                std::cout << "Got packet " << msg.msgid << std::endl;
        }
        read_some();
    }

    void read_some() {
        _serial.async_read_some(boost::asio::buffer(_buffer, SERIAL_BUF_SIZE),
                [&](boost::system::error_code ec, std::size_t len){_handler(ec, len);});
    }
private:
    boost::asio::io_service     _io;
    boost::asio::serial_port    _serial;
    char                        _buffer[SERIAL_BUF_SIZE];
};




int main() {
    Serial uart("/dev/ttyS0", 115200);
    return 0;
}