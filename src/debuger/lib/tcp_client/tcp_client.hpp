#ifndef SEU_UNIROBOT_DEBUGER_TCP_CLIENT_HPP
#define SEU_UNIROBOT_DEBUGER_TCP_CLIENT_HPP

#include <boost/asio.hpp>
#include <map>
#include <memory>
#include <thread>
#include "tcp.hpp"

class tcp_client
{
public:
    tcp_client(const std::string &addr, const int &port, tcp_callback tcb=nullptr);
    ~tcp_client();
    void start();
    void stop();
    void write(const tcp_command& cmd);
    void regist(const tcp_cmd_type &type, const tcp_data_dir &dir);
    bool is_connected() const
    {
        return is_connect_;
    }
    
private:
    void deliver(const tcp_command& cmd);
    boost::asio::ip::tcp::socket socket_;
    std::thread td_;
    std::string addr_;
    int port_;
    bool is_connect_;
    bool is_alive_;
    tcp_callback tcb_;
};

#endif
