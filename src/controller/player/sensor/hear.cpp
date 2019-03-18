#include "hear.hpp"
#include "configuration.hpp"

using boost::asio::ip::udp;
using namespace std;

boost::asio::io_service sh_service;

hear::hear() : sensor("hear"),
    socket_(sh_service, udp::endpoint(udp::v4(), CONF->get_config_value<short>("net.udp.team.port")))
{
}

bool hear::start()
{
    is_open_ = true;
    is_alive_ = true;
    td_ = std::move(std::thread([this]()
    {
        receive();
        sh_service.run();
    }));
    return true;
}

void hear::stop()
{
    is_open_ = false;
    is_alive_ = false;
    socket_.cancel();
    sh_service.stop();
}

void hear::receive()
{
    socket_.async_receive_from(boost::asio::buffer((char *)&pkt_, sizeof(comm_packet)), point_,
                               [this](boost::system::error_code ec, std::size_t bytes_recvd)
    {
        if (!ec && bytes_recvd > 0)
        {
            string recv_header;
            recv_header.append(pkt_.header, sizeof(comm_packet::header));
            if(recv_header==COMM_DATA_HEADER && pkt_.info.id != CONF->id())
            {
                //LOG<<pkt_.info.id<<'\t'<<pkt_.info.y<<ENDL;
                notify(SENSOR_HEAR);
            }
        }

        receive();
    });
}

hear::~hear()
{
    if (td_.joinable())
    {
        td_.join();
    }
}