#ifndef SEU_UNIROBOT_SERIAL_PORT_HANDLER_HPP
#define SEU_UNIROBOT_SERIAL_PORT_HANDLER_HPP

#include "comm/serial_port.hpp"
#include "class_exception.hpp"

class serial_port_handler
{
public:
    serial_port_handler(const std::string &dev_name, const int &baudrate, comm::ser_comm_callback cb=nullptr);
    void close();
    void run();
    void write(const uint8_t *data, const int &size);
    bool read(uint8_t *data, const int &size, const int &ms);
    void start_read(const int &size);
private:
    comm::serial_port serial_;
};

#endif