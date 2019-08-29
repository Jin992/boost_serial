//
// Created by jin on 8/16/19.
//

#include "MavProxy.h"
#define CUR_TIME_M_SEC (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())
namespace {
    /**
     * std::string make_magic_key(std::string const &key)
     * @brief - generate magic key
     * @param key - part of the key
     * @return string that contain generated key
     */
    std::string make_magic_key(std::string const &key) {
        char mdString[33];
        std::string to_transform(key + " this is magic");
        unsigned char digest[MD5_DIGEST_LENGTH];

        MD5((unsigned char*)to_transform.c_str(), to_transform.size(), (unsigned char*)&digest);
        for(int i = 0; i < 16; i++)
            sprintf(&mdString[i*2], "%02x", (unsigned int)digest[i]);
        return std::string(mdString);
    }

    /**
     * void decode_mavlink(char *buf, unsigned long buf_size, mavlink_message_t &msg)
     * @brief - mavlink decoder function
     * @param buf - mavlink message in binary representation
     * @param buf_size - size of buf
     * @param msg decoded mavlink packet
     */
    void decode_mavlink(char *buf, unsigned long buf_size, mavlink_message_t &msg) {
        mavlink_status_t mavlink_status;
        for (int i = 0; i < buf_size; i++) {
            if (mavlink_parse_char(0, (uint8_t )buf[i], &msg, &mavlink_status))
                break;
        }
    }
}

MavProxy::MavProxy(std::string const & dev_name, uint32_t baud_rate):_io(), _serial(_io, dev_name), _socket(_io)
{
    /// initialize serial port
    _init_serial(baud_rate);
    /// initialize udp socket
    _init_udp_socket("192.168.0.135", "25090");
    /// generate magic string
    _magic = make_magic_key("Radion");
}

/**
 * void MavProxy::run()
 * @brief launch io_service event loop
 */
void MavProxy::run() {
    /// start asynchronously read from serial
    _read_serial();
    /// start asynchronously read from udp socket
    _read_udp();
    /// launch io object in thread
    std::thread t([this](){ _io.run();});
    if (t.joinable())
        t.join();
}

/**
 * void MavProxy::_init_serial(uint32_t baud_rate)
 * @brief serial port initialization
 * @param baud_rate - serial port  baud rate
 */
void MavProxy::_init_serial(uint32_t baud_rate) {
    _serial.set_option( boost::asio::serial_port_base::parity() );	// default none
    _serial.set_option( boost::asio::serial_port_base::character_size( 8 ) );
    _serial.set_option( boost::asio::serial_port_base::stop_bits() );	// default one
    _serial.set_option( boost::asio::serial_port_base::baud_rate( baud_rate ) );
}

/**
 *
 */
void MavProxy::_init_udp_socket (const std::string &ip, const std::string &port){
    boost::asio::ip::udp::resolver resolver(_io);                                                                           /// initialize resolver
    boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(), ip, port);           /// setup protocol + ip + port query for resolver
    _remote_endpoint = *resolver.resolve(query);/// initialize endpoint with address resolver
    _socket = boost::asio::ip::udp::socket(_io);
    _socket.open(boost::asio::ip::udp::v4());
    if (!_socket.is_open()) {
        throw "UdpControlStream: failed to open socket";                                                                       /// temporary exception stub
    }
}
void MavProxy::_read_udp() {
    boost::asio::ip::udp::endpoint r_ep;
    //std::cout << "red udp" << std::endl;
    _socket.async_receive_from(boost::asio::buffer(_buf_in), r_ep,
                               [&](const boost::system::error_code &error, uint bytes_transferred){
                                   _read_udp_hdl(error, bytes_transferred);
                               });
}

void MavProxy::_read_udp_hdl(const boost::system::error_code &error, uint bytes_transferred) {
    if(error)
        std::cout << error.message() << std::endl;
    mavlink_message_t msg;
    decode_mavlink((char*)_buf_in.data(), bytes_transferred, msg);
    _serial.async_write_some(boost::asio::buffer(_buf_in.data(),bytes_transferred),
                             [&](const boost::system::error_code &error, std::size_t len){ _write_serial_hdl(error, len);});
}


void MavProxy::_write_serial_hdl(const boost::system::error_code &error, uint bytes_transferred) {
    if (error)
        std::cout << error.message() << std::endl;
    /// TODO thi part create small freezes while mission download/upload

    _read_udp();
}

//-----------------------------------------------------------------------------------------------------------------------------------------
void MavProxy::_read_serial() {
    //std::cout << "read serial " << std::endl;
    _serial.async_read_some(boost::asio::buffer(_buffer, SERIAL_BUF_SIZE),
                            [&](boost::system::error_code ec, std::size_t len){_read_serial_handler(ec, len);});
}

void MavProxy::_read_serial_handler(const boost::system::error_code &error, uint bytes_transferred) {
    //std::cout << "read serial hdl" << std::endl;
    mavlink_status_t status;
    bool state = true;
    for (std::size_t i = 0; i < bytes_transferred; i++){
        if (mavlink_parse_char(MAVLINK_COMM_0, _buffer[i], &_msg, &status)) {
            _write_udp(_msg);
        }
    }
    _read_serial();
}

void MavProxy::_write_udp(mavlink_message_t msg) {
    //std::cout << "write hdl" << std::endl;
    char tmp[512];
    uint32_t data_size = mavlink_msg_to_send_buffer((uint8_t *)tmp, &msg);
    std::shared_ptr<std::string> send_buf = std::make_shared<std::string>(_magic);
    send_buf->append(tmp, data_size);
    _socket.async_send_to(boost::asio::buffer(send_buf->data(), send_buf->size()), _remote_endpoint,
                          [this,send_buf](const boost::system::error_code& error, std::size_t bytes_transferred){
                              _write_udp_hdl(error, bytes_transferred, send_buf);});
}

void MavProxy::_write_udp_hdl(const boost::system::error_code& error, std::size_t bytes_transferred, std::shared_ptr<std::string> ptr) {
    if (error)
        std::cout << error.message() << std::endl;
    //std::cout << "write udp hdl" << std::endl;
}