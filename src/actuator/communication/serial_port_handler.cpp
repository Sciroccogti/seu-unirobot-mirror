#include "serial_port_handler.hpp"

using namespace std;
using namespace comm;

boost::asio::io_service ser_io_service;

serial_port_handler::serial_port_handler(const std::string &dev_name, const int &baudrate, ser_comm_callback cb)
    : serial_(ser_io_service, dev_name, baudrate, move(cb))
{
}

void serial_port_handler::run()
{
    ser_io_service.run();
}

void serial_port_handler::close()
{
    serial_.close();
    ser_io_service.stop();
}

void serial_port_handler::write(const uint8_t *data, const int &size)
{
    serial_.write(data, size);
}

bool serial_port_handler::read(uint8_t *data, const int &size, const int &ms)
{
    serial_.read(data, size, ms);
    while(!serial_.read_complete()&&!serial_.timeout());
    return (!serial_.timeout());
}

void serial_port_handler::start_read(const int &size)
{
    serial_.start_read(size);
}