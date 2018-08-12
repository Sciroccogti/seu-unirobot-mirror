//
// Created by lcseu on 18-8-10.
//

#include "tcp_client_handler.hpp"

using namespace comm;
using namespace std;

boost::asio::io_service io_service_;

tcp_client_handler::tcp_client_handler(const string &host, const int &port, tcp_comm_callback cb)
    :client_(io_service_, host, port, std::move(cb))
{
}

bool tcp_client_handler::write(const comm::tcp_packet::tcp_command &cmd)
{
    client_.write(cmd);
}

void tcp_client_handler::write(const comm::tcp_packet::tcp_cmd_type &type, const int &size, const char *data)
{
    client_.write(type, size, data);
}

void tcp_client_handler::run()
{
    io_service_.run();
}

void tcp_client_handler::close()
{
    client_.close();
    io_service_.stop();
}

void tcp_client_handler::regist(const comm::tcp_packet::tcp_cmd_type &type, const comm::tcp_packet::tcp_data_dir &dir)
{
    tcp_packet::tcp_command cmd;
    cmd.type = tcp_packet::REG_DATA;
    cmd.size = tcp_packet::tcp_cmd_type_size + tcp_packet::tcp_data_dir_size;
    std::memcpy(cmd.data, &type, tcp_packet::tcp_cmd_type_size);
    std::memcpy(cmd.data+tcp_packet::tcp_cmd_type_size, &dir, tcp_packet::tcp_data_dir_size);
    client_.write(cmd);
}