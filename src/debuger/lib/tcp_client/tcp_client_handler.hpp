//
// Created by lcseu on 18-8-10.
//

#ifndef SEU_UNIROBOT_DEBUGER_TCP_CLIENT_HANDLER_HPP
#define SEU_UNIROBOT_DEBUGER_TCP_CLIENT_HANDLER_HPP

#include <memory>
#include <thread>
#include "comm/tcp_client.hpp"

class tcp_client_handler
{
public:
    tcp_client_handler(const std::string &host, const int &port, comm::net_comm_callback cb = nullptr);
    ~tcp_client_handler();
    void run();
    void start();
    void close();
    void regist(const comm::tcp_packet::tcp_cmd_type &type, const comm::tcp_packet::tcp_data_dir &dir);
    bool is_connected() const { return client_.is_connected(); }
    bool write(const comm::tcp_packet::tcp_command &cmd);
    void write(const comm::tcp_packet::tcp_cmd_type &type, const int &size, const char *data);
private:
    comm::tcp_client client_;
    std::thread td_;
};

#endif //SEU_UNIROBOT_TCP_CLIENT_HANDLER_HPP
