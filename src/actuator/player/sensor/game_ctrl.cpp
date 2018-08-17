#include "game_ctrl.hpp"
#include <boost/asio.hpp>
#include "configuration.hpp"

using namespace comm;
using namespace std;

boost::asio::io_service gc_io_service;

game_ctrl::game_ctrl(const sub_ptr &s):sensor("game_ctrl"), server_(gc_io_service, 
udp::endpoint(boost::asio::ip::address_v4::from_string(CONF.get_config_value<string>("net.udp.addr")), CONF.get_config_value<short>("net.udp.game_ctrl.recv_port")), 
gc_data_size, GAMECONTROLLER_STRUCT_HEADER, bind(&game_ctrl::data_handler, this, placeholders::_1, placeholders::_2, placeholders::_3))
{
    attach(s);
    str_.resize(gc_data_size);
    data_ = (RoboCupGameControlData*)(str_.data());
}

bool game_ctrl::open()
{
    is_alive_ = true;
    return true;
}

bool game_ctrl::start()
{
    this->open();
    td_ = std::thread(bind(&game_ctrl::run, this));
    return true;
}

void game_ctrl::run()
{
    gc_io_service.run();
}

void game_ctrl::stop()
{
    server_.close();
    gc_io_service.stop();
    is_alive_ = false;
}

void game_ctrl::data_handler(const char* data, const int& size, const int& type)
{
    if(size==gc_data_size)
    {
        str_.assign(data, size);
        notify();
    }
}

game_ctrl::~game_ctrl()
{
    if(td_.joinable()) td_.join();
}
