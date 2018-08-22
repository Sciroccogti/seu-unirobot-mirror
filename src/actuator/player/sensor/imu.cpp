#include "imu.hpp"
#include <boost/asio.hpp>
#include "configuration.hpp"

using namespace std;
using namespace boost::asio;

boost::asio::io_service imu_service;
unsigned char imu_header[] = {0xA5, 0x5A};
imu::imu(const sub_ptr& s): sensor("imu"), serial_(imu_service)
{
    attach(s);
    str_.resize(imu_data_size);
    data_ = (imu_data*)(str_.data());
}

bool imu::open()
{
    try
    {
        serial_.open(CONF.get_config_value<string>("hardware.imu.dev_name"));
        serial_.set_option(boost::asio::serial_port::baud_rate(CONF.get_config_value<unsigned int>("hardware.imu.baudrate")));
        serial_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        serial_.set_option(boost::asio::serial_port::character_size(8));
        return true;
    }
    catch(exception &e)
    {
        LOG<<LOG_ERROR<<e.what()<<"\n";
        return false;
    }
}

bool imu::start()
{
    if(!this->open()) return false;
    is_open_ = true;
    is_alive_ = true;
    short pitch, roll, yaw;
    td_ = std::move(thread([this]()
    {
        boost::system::error_code ec;
        const int max_len = 64;
        unsigned char len=0;
        unsigned char buff[max_len];
        while(is_alive_)
        {
            serial_.read_some(boost::asio::buffer(buff, 1), ec);
            if(ec)
            {
                LOG<<LOG_ERROR<<ec.message()<<"\n";
                break;
            }
            if(buff[0] != imu_header[0]) continue;
            serial_.read_some(boost::asio::buffer(buff+1, 1), ec);
            if(ec)
            {
                LOG<<LOG_ERROR<<ec.message()<<"\n";
                break;
            }
            if(buff[1] != imu_header[1]) continue;
            serial_.read_some(boost::asio::buffer(buff+2, 1), ec);
            if(ec)
            {
                LOG<<LOG_ERROR<<ec.message()<<"\n";
                break;
            }
            len = buff[2];
            serial_.read_some(boost::asio::buffer(buff+3, (size_t)len), ec);
            if(ec)
            {
                LOG<<LOG_ERROR<<ec.message()<<"\n";
                break;
            }
            notify();
        }
    }));
    return true;
}

void imu::stop()
{
    serial_.close();
    is_alive_ = false;
    is_open_ = false;
}

imu::~imu()
{
    if(td_.joinable()) td_.join();
}
