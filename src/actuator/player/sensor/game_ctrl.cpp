#include "game_ctrl.hpp"
#include <boost/asio.hpp>
#include "configuration.hpp"

using namespace comm;
using namespace std;

boost::asio::io_service gc_io_service;

game_ctrl::game_ctrl(const sub_ptr &s):sensor("game_ctrl"), server_(gc_io_service, 
udp::endpoint(boost::asio::ip::address_v4::from_string(CONF.get_config_value<string>("net.udp.game_ctrl.addr")), CONF.get_config_value<short>("net.udp.game_ctrl.recv_port")), 
gc_data_size, GAMECONTROLLER_STRUCT_HEADER, 4, bind(&game_ctrl::data_handler, this, placeholders::_1, placeholders::_2, placeholders::_3))
{
    attach(s);
    str_.resize(gc_data_size);
    data_ = (RoboCupGameControlData*)(str_.data());
}

void game_ctrl::data_handler(const char* data, const int& size, const int& type)
{
    if(size==gc_data_size)
    {
        str_.assign(data, size);
        notify();
    }
}

void game_ctrl::close()
{
    if(is_open_)
    {
        server_.close();
        gc_io_service.stop();
    }
    is_open_ = false;
    is_alive_ = false;
    std::cout<<"\033[32mgame_ctrl closed!\033[0m\n";
}

bool game_ctrl::open()
{
    is_open_ = true;
    is_alive_ = true;
    return true;
}

void game_ctrl::run()
{
    gc_io_service.run();
}

bool game_ctrl::start()
{
    this->open();
    td_ = std::thread(bind(&game_ctrl::run, this));
    return true;
}

game_ctrl::~game_ctrl()
{
    if(td_.joinable()) td_.join();
}
