#include "imu.hpp"
#include <boost/asio.hpp>
#include "configuration.hpp"

using namespace std;

boost::asio::io_service imu_io_service;

imu::imu(const sub_ptr& s)
    : sensor("imu"), serial_(imu_io_service, CONF.get_config_value<string>("serial.imu.dev_name"), CONF.get_config_value<int>("serial.imu.baudrate"))
{
    attach(s);
    str_.resize(imu_data_size);
    data_ = (imu_data*)(str_.data());
}

bool imu::open()
{
    is_open_ = true;
    is_alive_ = true;
    return true;
}

bool imu::start()
{
    this->open();
    td_ = thread(bind(&imu::run, this));
    return true;
}

void imu::run()
{
    imu_io_service.run();
}

void imu::stop()
{
    if(is_open_)
    {
        serial_.close();
        imu_io_service.stop();
    }
    is_open_ = false;
    is_alive_ = false;
}

void imu::data_handler(const char* data, const int& size, const int& type)
{
    if(size==imu_data_size)
    {
        str_.assign(data, size);
        notify();
    }
}

imu::~imu()
{
    if(td_.joinable()) td_.join();
}
