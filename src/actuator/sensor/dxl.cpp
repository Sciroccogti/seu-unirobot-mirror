#include "dxl.hpp"
#include "configuration.hpp"

#define ADDR_ID     7
#define ADDR_TORQ   64
#define ADDR_LED    65
#define ADDR_GPOS   116
#define ADDR_PPOS   132
#define ADDR_VOLT   144

#define SIZE_ID     1
#define SIZE_TORQ   1
#define SIZE_LED    1
#define SIZE_GPOS   4
#define SIZE_PPOS   4
#define SIZE_VOLT   2

using namespace std;
using namespace dynamixel;

dxl::dxl(const sub_ptr& s): timer(CONF.get_config_value<int>("serial.dxl.period")),
    period_(CONF.get_config_value<int>("serial.dxl.period")), p_count_(0)
    
{
    portHandler_ = PortHandler::getPortHandler(CONF.get_config_value<string>("serial.dxl.dev_name").c_str());
    packetHandler_ = PacketHandler::getPacketHandler(CONF.get_config_value<float>("serial.dxl.version"));
}

dxl::~dxl()
{

}

bool dxl::open()
{
    if(portHandler_->openPort())
    {
        if(portHandler_->setBaudRate(CONF.get_config_value<int>("serial.dxl.baudrate")))
        {
            is_open_ = true;
            sensor::is_alive_ = true;
            timer::is_alive_ = true;
            return true;
        }
    }
    return false;
}

void dxl::close()
{
    if(is_open_)
    {
        is_open_ = false;
        sensor::is_alive_ = false;
        timer::is_alive_ = false;
        portHandler_->closePort();
    }
}

bool dxl::start()
{
    return this->open();
}

void dxl::run()
{
    if(timer::is_alive_)
    {
        if((p_count_*period_%1000) == 0)
        {
        }
        if((p_count_*period_%10000) == 0)
        {
        }
        p_count_++;
    }
}
