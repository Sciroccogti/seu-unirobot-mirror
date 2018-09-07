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
    socket_.async_receive_from(boost::asio::buffer((char*)&p_info_, player_info_size), point_,
    [this](boost::system::error_code ec, std::size_t bytes_recvd)
    {
        if (!ec && bytes_recvd > 0)
        {
            notify(SENSOR_HEAR);
        }
        receive();
    });
}

void hear::send(const player_info &pinfo)
{
    if(!is_alive_) return;
    try
    {
        socket_.async_send_to(boost::asio::buffer((char*)(&pinfo), player_info_size), point_,
                                   [this](boost::system::error_code ec, std::size_t bytes_sent){});
    }
    catch(exception &e)
    {
        std::cout<<e.what()<<"\n";
    }
}

hear::~hear()
{
    if(td_.joinable()) td_.join();
}