#ifndef SEU_UNIROBOT_REMOTE_HPP
#define SEU_UNIROBOT_REMOTE_HPP

#include <thread>
#include "sensor.hpp"
#include "tcp_server/tcp_server_handler.hpp"

class remote: public sensor
{
public:
    remote(const sub_ptr& s);
    ~remote();
    
    bool start();
    void run();
    bool open();
    void close();
    
    comm::tcp_packet::remote_data data() const
    {
        return data_;
    }
private:
    comm::tcp_packet::remote_data data_;
    std::thread td_;
};

#endif