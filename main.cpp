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
#include "MavlinkParam.h"
#include <mavlink/include/mavlink/v2.0/ardupilotmega/mavlink_msg_video_stream_bitrate.h>

#include <chrono>
#include "Modem.h"

#define CUR_TIME_M_SEC (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())

#define SERIAL_BUF_SIZE 32768

typedef std::vector<MavlinkParam>::iterator ParamIterator_t;

std::queue<std::pair<boost::array<int8_t, 1024>, std::size_t> > in_queue;
std::mutex                               in_q_mtx;
std::condition_variable                  in_q_cond;

typedef std::pair<uint8_t *, std::size_t>                mav_buf_t;
std::mutex              storage_mutex;
std::condition_variable cond;

typedef struct uav_config_s {
    u_int16_t               video_port;
    u_int16_t               mav_proxy_port;
    u_int16_t               control_port;
    std::string             video_dev;
    std::atomic<int>        video_fps;

    std::atomic<u_int16_t>  video_width;
    std::atomic<u_int16_t>  video_height;

    std::atomic<u_int16_t>  latency_threshold;
    std::atomic<u_int8_t>   bt_increase;
    std::atomic<u_int8_t>   bt_decrease;
    std::atomic<int32_t>    bt_period;

    std::string             host_ip;

    std::atomic<uint8_t>    latency_min_upd;
    std::atomic<uint8_t>    latency_av_upd;
    std::atomic<uint32_t>   latency_min_ts;
    std::atomic<uint32_t>   latency_av_ts;
    std::atomic<int64_t>    reconnect_timeout;

}              uav_config_t;

namespace {
    std::string make_magic_key(std::string const &key) {
        char mdString[33];
        std::string to_transform(key + " this is magic");
        unsigned char digest[MD5_DIGEST_LENGTH];

        MD5((unsigned char*)to_transform.c_str(), to_transform.size(), (unsigned char*)&digest);
        for(int i = 0; i < 16; i++)
            sprintf(&mdString[i*2], "%02x", (unsigned int)digest[i]);
        return std::string(mdString);
    }

    void decode_mavlink(char *buf, unsigned long buf_size, mavlink_message_t &msg) {
        int chan = 0;
        mavlink_status_t mavlink_status;
        for (int i = 0; i < buf_size; i++) {
            if (mavlink_parse_char(chan, (uint8_t )buf[i], &msg, &mavlink_status)){
                break;
            }
        }
    }
}

class Rpi {
public:
    Rpi(uav_config_t &config): _config(config) {
        _initialize_params();
    }

    void set_mavlink_emitter(std::function<void (const mavlink_message_t &)> func) {
        _send_mavlink = func;
    }

    void calc(){}
    mavlink_message_t     generate_param_value(int sys_id, int comp_id, MavlinkParam &param) {
        mavlink_message_t       msg = {0};
        mavlink_param_value_t   val_pkt = {0};

        strcpy(val_pkt.param_id, param.get_param_id());
        val_pkt.param_value = param.get_param_value();
        val_pkt.param_type = param.get_param_type();
        val_pkt.param_index = param.get_param_index();
        val_pkt.param_count = param.get_param_count();

        mavlink_msg_param_value_encode(sys_id, comp_id, &msg, &val_pkt);
        return  msg;
    }
    mavlink_message_t    generate_cam_bitrate(std::atomic <float>  &bitrate){
        mavlink_video_stream_bitrate_t bt;
        mavlink_message_t msg;
        bt.Bitrate = bitrate;
        mavlink_msg_video_stream_bitrate_encode(2, 1, &msg, &bt);
        return  msg;
    }
    mavlink_message_t    generate_heartbeat() {
        mavlink_message_t msg;
        mavlink_heartbeat_t hrt;

        /// filling heartbeat message
        hrt.type = 	MAV_TYPE_SUBMARINE; // vehicle type
        hrt.system_status = MAV_STATE_ACTIVE; // current vehicle status
        hrt.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA; // vehicle autopilot type
        hrt.custom_mode = 11;
        hrt.base_mode = 89; // ?
        /// ecncode heartbeat to mavlink
        mavlink_msg_heartbeat_encode(2, 1, &msg, &hrt);
        return  msg;
    }

    mavlink_message_t generate_modem_rssi() {
        return _modem.get_modem_data();
    }

    void  process_param_request(mavlink_message_t msg){
        switch(msg.msgid) {
            case MAVLINK_MSG_ID_PARAM_SET: {
                _param_set(msg);
                break;
            }
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                _param_list(msg);
                break;
            }
            case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
                _param_read(msg);
                break;
            }

            default:
                break;
        }
    }

private:
    void            _initialize_params(){
        _params.emplace_back(MavlinkParam("UAV_THD", static_cast<float>(_config.latency_threshold), 0, 10, 	MAV_PARAM_TYPE_REAL32));
        _params.emplace_back(MavlinkParam("UAV_BTINC", static_cast<float>(_config.bt_increase), 1, 10, MAV_PARAM_TYPE_REAL32));
        _params.emplace_back(MavlinkParam("UAV_BTDEC", static_cast<float>(_config.bt_decrease), 2, 10, MAV_PARAM_TYPE_REAL32));
        _params.emplace_back(MavlinkParam("UAV_BTPRD", static_cast<float>(_config.bt_period), 3, 10,	MAV_PARAM_TYPE_REAL32));

        _params.emplace_back(MavlinkParam("UAV_LTAVTS", static_cast<float>(_config.latency_av_ts), 4, 10,	MAV_PARAM_TYPE_REAL32));
        _params.emplace_back(MavlinkParam("UAV_LTAVUPD", static_cast<float>(_config.latency_av_upd), 5, 10,	MAV_PARAM_TYPE_REAL32));
        _params.emplace_back(MavlinkParam("UAV_LTMINTS", static_cast<float>(_config.latency_min_ts), 6, 10,	MAV_PARAM_TYPE_REAL32));
        _params.emplace_back(MavlinkParam("UAV_LTMINUPD", static_cast<float>(_config.latency_min_upd), 7, 10,	MAV_PARAM_TYPE_REAL32));
        _params.emplace_back(MavlinkParam("UAV_VRES", static_cast<float>(_config.video_height), 8, 10,	MAV_PARAM_TYPE_REAL32));
        _params.emplace_back(MavlinkParam("UAV_VRET", 0.0, 9, 10,	MAV_PARAM_TYPE_REAL32));
    }

    void            _param_read(mavlink_message_t &msg){
        mavlink_param_request_read_t read_pkt;

        mavlink_msg_param_request_read_decode(&msg, &read_pkt);
        if (read_pkt.target_system == 2 && read_pkt.target_component == 1) {
            auto it = _find_param(read_pkt.param_id);
            if (it == _params.end())
                return;
            mavlink_message_t res_msg = generate_param_value(2, 1, *it);
            _send_mavlink(res_msg);
        }
    }
    void            _param_list(mavlink_message_t &msg){
        mavlink_param_request_list_t list_pkt;

        mavlink_msg_param_request_list_decode(&msg, &list_pkt);
        std::cout << "TARGET SYSTEM " << (int)list_pkt.target_system << std::endl;
        if (list_pkt.target_system == 2 && list_pkt.target_component == 0) {
            for (auto it : _params) {
                mavlink_message_t res_msg = generate_param_value(2, 1, it);
                _send_mavlink(res_msg);
            }
        }
    }
    void            _param_set(mavlink_message_t &msg){
        mavlink_param_set_t set_pkt;
        /// extract param_set message from mavlink payload
        mavlink_msg_param_set_decode(&msg, &set_pkt);
        /// check if this set_param msg destinaten to current vehicle
        if (set_pkt.target_system == 2 && set_pkt.target_component == 1) {
            auto it = _find_param(set_pkt.param_id);
            if (it == _params.end())
                return;
            it->set_param_value(set_pkt.param_value);
            if (strcmp(it->get_param_id(), "UAV_LTMINTS") == 0) {
                _config.latency_min_ts = static_cast<int>(it->get_param_value());
            }
            else if (strcmp(it->get_param_id(), "UAV_LTMINUPD") == 0) {
                _config.latency_min_upd = static_cast<int>(it->get_param_value());
            }
            else if (strcmp(it->get_param_id(), "UAV_LTAVTS") == 0) {
                _config.latency_av_ts = static_cast<int>(it->get_param_value());
            }
            else if (strcmp(it->get_param_id(), "UAV_LTAVUPD") == 0) {
                _config.latency_av_upd = static_cast<int>(it->get_param_value());
            }
            else if (strcmp(it->get_param_id(), "UAV_BTINC") == 0) {
                _config.bt_increase = static_cast<int>(it->get_param_value());
            }
            else if (strcmp(it->get_param_id(), "UAV_BTDEC") == 0) {
                _config.bt_decrease = static_cast<int>(it->get_param_value());
            }
            else if (strcmp(it->get_param_id(), "UAV_BTPRD") == 0) {
                _config.bt_period = static_cast<int>(it->get_param_value());
            }
            else if (strcmp(it->get_param_id(), "UAV_THD") == 0) {
                _config.latency_threshold = static_cast<int>(it->get_param_value());
            }
            mavlink_message_t res_msg = generate_param_value(2, 1, *it);
            _send_mavlink(res_msg);
        }
    }
    ParamIterator_t _find_param(char *param) {
        auto it = std::find_if(_params.begin(), _params.end(), [param](MavlinkParam &p){
            if (strcmp(param, p.get_param_id()) == 0)
                return true;
            return false;
        });
        return it;
    }
private:
    std::function<void (const mavlink_message_t &)> _send_mavlink;
    uav_config_t                                    &_config;
    std::vector<MavlinkParam>                       _params;
    Modem                                           _modem;
};

uav_config_t config;
class MavProxy {
public:
    MavProxy(std::string const & dev_name, uint32_t baud_rate):_io(), _serial(_io, dev_name), _socket(_io), _computer(config), _bt(1.7), _timeout(CUR_TIME_M_SEC), _d_timer(_io)
    {
        /// initialize serial port
        _init_serial(baud_rate);
        /// initialize udp socket
        _init_udp_socket();
        /// generate magic string
        _magic = make_magic_key("Radion");
    }

    void run() {
        _onboard_computer_mavlink_transfer();
        _read_serial();
        _read_udp();
        std::thread t([this](){ _io.run();});
        if (t.joinable())
            t.join();
    }


private:
    void _onboard_computer_mavlink_transfer() {
        _d_timer.expires_from_now(boost::posix_time::seconds(1));
        _d_timer.async_wait([this](const boost::system::error_code& e){ _on_timer(e);});
        _computer.set_mavlink_emitter([this](mavlink_message_t const &msg){ _write_udp(msg);});
    }

    void _on_timer(const boost::system::error_code& e) {
        if (_d_timer.expires_at() <= boost::asio::deadline_timer::traits_type::now()) {
            _d_timer.cancel();
            mavlink_message_t h = _computer.generate_heartbeat();
            _write_udp(h);
            mavlink_message_t bt = _computer.generate_cam_bitrate(_bt);
            _write_udp(bt);
            mavlink_message_t modem = _computer.generate_modem_rssi();
            _write_udp(modem);
            _d_timer.expires_from_now(boost::posix_time::seconds(1));
            _d_timer.async_wait([this](const boost::system::error_code& e){ _on_timer(e);});
        }
    }

    void _init_serial(uint32_t baud_rate) {
        _serial.set_option( boost::asio::serial_port_base::parity() );	// default none
        _serial.set_option( boost::asio::serial_port_base::character_size( 8 ) );
        _serial.set_option( boost::asio::serial_port_base::stop_bits() );	// default one
        _serial.set_option( boost::asio::serial_port_base::baud_rate( baud_rate ) );
    }

    void _init_udp_socket() {
        boost::asio::ip::udp::resolver resolver(_io);                                                                           /// initialize resolver
        boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(), "192.168.0.135", "25090");           /// setup protocol + ip + port query for resolver
        _remote_endpoint = *resolver.resolve(query);/// initialize endpoint with address resolver
        _socket = boost::asio::ip::udp::socket(_io);
        _socket.open(boost::asio::ip::udp::v4());
        if (!_socket.is_open()) {
            throw "UdpControlStream: failed to open socket";                                                                       /// temporary exception stub
        }
    }
    void _read_udp() {
        boost::asio::ip::udp::endpoint r_ep;
        //std::cout << "red udp" << std::endl;
        _socket.async_receive_from(boost::asio::buffer(_buf_in), r_ep,
                [&](const boost::system::error_code &error, uint bytes_transferred){
                                            _read_udp_hdl(error, bytes_transferred);
        });
    }

    void _read_udp_hdl(const boost::system::error_code &error, uint bytes_transferred) {
        if(error)
            std::cout << error.message() << std::endl;
        //std::cout << "read udp hdl" << std::endl;

        _serial.async_write_some(boost::asio::buffer(_buf_in.data(),bytes_transferred),
                                 [&](const boost::system::error_code &error, std::size_t len){ _write_serial_hdl(error, len);});
    }


    void _write_serial_hdl(const boost::system::error_code &error, uint bytes_transferred) {
        if (error)
            std::cout << error.message() << std::endl;
        //std::cout << "write serial hdl" << std::endl;
        /// TODO thi part create small freezes while mission download/upload
        mavlink_message_t msg;
        boost::array<int8_t, 1024> tmp = _buf_in;
        decode_mavlink((char*)tmp.data(), bytes_transferred, msg);
        _computer.process_param_request(msg);
        _read_udp();
    }

//-----------------------------------------------------------------------------------------------------------------------------------------
    void _read_serial() {
        //std::cout << "read serial " << std::endl;
        _serial.async_read_some(boost::asio::buffer(_buffer, SERIAL_BUF_SIZE),
                                [&](boost::system::error_code ec, std::size_t len){_read_serial_handler(ec, len);});
    }

    void _read_serial_handler(const boost::system::error_code &error, uint bytes_transferred) {
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

    void _write_udp(mavlink_message_t msg) {
        //std::cout << "write hdl" << std::endl;
        char tmp[512];
        uint32_t data_size = mavlink_msg_to_send_buffer((uint8_t *)tmp, &msg);
        std::shared_ptr<std::string> send_buf = std::make_shared<std::string>(_magic);
        send_buf->append(tmp, data_size);
        _socket.async_send_to(boost::asio::buffer(send_buf->data(), send_buf->size()), _remote_endpoint,
                              [this,send_buf](const boost::system::error_code& error, std::size_t bytes_transferred){
                                  _write_udp_hdl(error, bytes_transferred, send_buf);});
    }

    void _write_udp_hdl(const boost::system::error_code& error, std::size_t bytes_transferred, std::shared_ptr<std::string> ptr) {
        if (error)
            std::cout << error.message() << std::endl;
        //std::cout << "write udp hdl" << std::endl;
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
    Rpi                             _computer;
    std::atomic<float>              _bt;
    int64_t                         _timeout;
    boost::asio::deadline_timer     _d_timer;
};



int main() {
    MavProxy proxy("/dev/ttyS0", 115200);
    proxy.run();
    std::cout << "Finish" << std::endl;
    return 0;
}