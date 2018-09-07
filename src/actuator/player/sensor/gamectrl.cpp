#include "gamectrl.hpp"
#include "configuration.hpp"

using namespace std;
using boost::asio::ip::udp;

boost::asio::io_service gc_service;

gamectrl::gamectrl(): sensor("gamectrl"), timer(CONF->get_config_value<int>("net.udp.gamectrl.send_period")),
        recv_socket_ (gc_service, udp::endpoint(udp::v4(), CONF->get_config_value<short>("net.udp.gamectrl.recv_port"))),
        send_socket_ (gc_service, udp::endpoint(udp::v4(), 0)), /*boost::asio::ip::address::from_string(CONF->get_config_value<string>("net.udp.addr"))*/
        send_point_(udp::v4(), CONF->get_config_value<unsigned short>("net.udp.gamectrl.send_port"))
{
    ret_data_.version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
    ret_data_.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
    ret_data_.player = static_cast<uint8_t>(CONF->id());
    ret_data_.team = CONF->get_config_value<uint8_t>("team_number");
    send_socket_.set_option(boost::asio::socket_base::broadcast(true));
}

bool gamectrl::start()
{
    is_open_ = true;
    sensor::is_alive_ = true;
    timer::is_alive_ = true;
    td_ = std::move(std::thread([this]()
    {
        this->receive();
        gc_service.run();
    }));
    start_timer();
    return true;
}

void gamectrl::receive()
{
    recv_socket_.async_receive_from(boost::asio::buffer((char*)&data_, gc_data_size), recv_point_,
    [this](boost::system::error_code ec, std::size_t bytes_recvd)
    {
        if (!ec && bytes_recvd > 0)
        {
            if(strcmp(GAMECONTROLLER_STRUCT_HEADER, data_.header))
                notify(SENSOR_GC);
        }
        receive();
    });
}

void gamectrl::run()
{
    if(timer::is_alive_)
    {
        try
        {
            send_socket_.async_send_to(boost::asio::buffer((char*)(&ret_data_), sizeof(RoboCupGameControlReturnData)), send_point_,
                          [this](boost::system::error_code ec, std::size_t bytes_sent){});
        }
        catch(exception &e)
        {
            std::cout<<e.what()<<"\n";
        }
    }
}

void gamectrl::stop()
{
    is_open_ = false;
    timer::is_alive_ = false;
    sensor::is_alive_ = false;
    delete_timer();
    recv_socket_.cancel();
    send_socket_.cancel();
    gc_service.stop();
}

gamectrl::~gamectrl()
{
    if(td_.joinable()) td_.join();
}
