#include "gamectrl.hpp"
#include <boost/asio.hpp>
#include "configuration.hpp"

using namespace comm;
using namespace std;

boost::asio::io_service gc_io_service;

gamectrl::gamectrl(const sub_ptr &s):sensor("gamectrl"), server_(gc_io_service, 
udp::endpoint(boost::asio::ip::address_v4::from_string(CONF.get_config_value<string>("net.udp.addr")), CONF.get_config_value<short>("net.udp.gamectrl.recv_port")), 
gc_data_size, GAMECONTROLLER_STRUCT_HEADER, bind(&gamectrl::data_handler, this, placeholders::_1, placeholders::_2, placeholders::_3))
{
    attach(s);
    str_.resize(gc_data_size);
    data_ = (RoboCupGameControlData*)(str_.data());
}

bool gamectrl::open()
{
    is_alive_ = true;
    return true;
}

bool gamectrl::start()
{
    this->open();
    td_ = std::thread(bind(&gamectrl::run, this));
    return true;
}

void gamectrl::run()
{
    gc_io_service.run();
}

void gamectrl::stop()
{
    server_.close();
    gc_io_service.stop();
    is_alive_ = false;
}

void gamectrl::data_handler(const char* data, const int& size, const int& type)
{
    if(size==gc_data_size)
    {
        str_.assign(data, size);
        notify();
    }
}

gamectrl::~gamectrl()
{
    if(td_.joinable()) td_.join();
}
