#include "gamectrl.hpp"
#include "configuration.hpp"

using namespace std;
using boost::asio::ip::udp;

boost::asio::io_service gc_service;

gamectrl::gamectrl(const sub_ptr &s):sensor("gamectrl"), timer(CONF.get_config_value<int>("net.udp.gamectrl.send_period")),
        ret_point_(boost::asio::ip::address_v4::from_string(CONF.get_config_value<string>("net.udp.addr")), CONF.get_config_value<unsigned short>("net.udp.gamectrl.send_port"))
{
    attach(s);
    str_.resize(gc_data_size);
    data_ = (RoboCupGameControlData*)(str_.data());
    ret_data_.version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
    ret_data_.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
    ret_data_.player = static_cast<uint8_t>(CONF.id());
    ret_data_.team = CONF.get_config_value<uint8_t>("team_number");
}

bool gamectrl::open()
{
    try
    {
        socket_ = make_shared<udp::socket>(gc_service, udp::endpoint(udp::v4(), CONF.get_config_value<unsigned short>("net.udp.gamectrl.recv_port")));
        ret_socket_ = make_shared<udp::socket>(gc_service, udp::endpoint(udp::v4(), 0));
        ret_socket_->set_option(boost::asio::socket_base::broadcast(true));
        return true;
    }
    catch (exception &e)
    {
        std::cout<<e.what()<<"\n";
        return false;
    }
}

bool gamectrl::start()
{
    if(!this->open()) return false;
    is_open_ = true;
    sensor::is_alive_ = true;
    timer::is_alive_ = true;
    td_ = std::move(std::thread([this]()
    {
        boost::system::error_code ec;
        udp::endpoint remote_point;
        char buff[gc_data_size];
        while(sensor::is_alive_)
        {
            if(!socket_->available()) continue;
            memset(buff, 0, gc_data_size);
            socket_->receive_from(boost::asio::buffer(buff, gc_data_size), remote_point);
            if(ec) break;
        }
    }));
    start_timer();
    return true;
}

void gamectrl::run()
{
    if(timer::is_alive_)
    {
        try
        {
            ret_socket_->send_to(boost::asio::buffer((char*)(&ret_data_), sizeof(RoboCupGameControlReturnData)), ret_point_);
        }
        catch(exception &e)
        {
            std::cout<<e.what()<<"\n";
        }
    }
}

void gamectrl::stop()
{
    timer::is_alive_ = false;
    sensor::is_alive_ = false;
    delete_timer();
    socket_->close();
    ret_socket_->close();
}

gamectrl::~gamectrl()
{
    if(td_.joinable()) td_.join();
}
