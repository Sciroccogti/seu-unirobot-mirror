#ifndef TCP_SERVER_HANDLER_HPP
#define TCP_SERVER_HANDLER_HPP

#include <thread>
#include <mutex>
#include "comm/tcp_server.hpp"
#include "singleton.hpp"

class tcp_server_handler: public singleton<tcp_server_handler>
{
public:
    tcp_server_handler();
    ~tcp_server_handler();
    void data_handler(const char *data, const int &size, const int &type);
    void write(const comm::tcp_packet::tcp_cmd_type &type, const int &size, const char *data);
    void write(const comm::tcp_packet::tcp_command &cmd);
    void close();
    void start();
    
    comm::tcp_packet::remote_data r_data() const
    {
        std::lock_guard<std::mutex> lk(tcp_mutex_);
        return r_data_;
    }
    mutable std::mutex tcp_mutex_;
protected:
    void run();
private:
    comm::tcp_server server_;
    std::thread td_;
    comm::tcp_packet::remote_data r_data_;
};

#define TCP_SERVER tcp_server_handler::get_singleton()
#endif