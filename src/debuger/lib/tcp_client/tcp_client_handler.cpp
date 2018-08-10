//
// Created by lcseu on 18-8-10.
//

#include "tcp_client_handler.hpp"

using namespace comm;
using namespace std;

boost::asio::io_service io_service_;

tcp_client_handler::tcp_client_handler(const string &host, const int &port)
    :client_(io_service_, host, port)
{
}

bool tcp_client_handler::write(const comm::tcp_packet::tcp_command &cmd)
{
    client_.write(cmd);
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

void tcp_client_handler::regist(comm::tcp_packet::tcp_cmd_type type, const bool &apply)
{
    tcp_packet::tcp_command cmd;
    cmd.type = apply?tcp_packet::APPLY_DATA:tcp_packet::SUPPLY_DATA;
    cmd.size = sizeof(tcp_packet::tcp_cmd_type);
    std::memcpy(cmd.data, reinterpret_cast<char*>(&type),cmd.size);
    client_.write(cmd);
}