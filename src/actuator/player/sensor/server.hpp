#ifndef SEU_UNIROBOT_ACTUATOR_DEBUGER_HPP
#define SEU_UNIROBOT_ACTUATOR_DEBUGER_HPP

#include <thread>
#include <mutex>
#include "comm/tcp_server.hpp"
#include "sensor/sensor.hpp"

class server: public sensor
{
public:
    server(const sub_ptr& s);
    ~server();
    void data_handler(const char *data, const int &size, const int &type);
    
    void write(const comm::tcp_packet::tcp_cmd_type &type, const int &size, const char *data);
    void write(const comm::tcp_packet::tcp_command &cmd);
    
    bool open();
    bool start();
    void run();
    void stop();
    
    comm::tcp_packet::remote_data r_data() const
    {
        return r_data_;
    }

private:
    comm::tcp_server server_;
    std::thread td_;
    comm::tcp_packet::remote_data r_data_;
};

#endif