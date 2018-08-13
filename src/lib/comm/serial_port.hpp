#ifndef SEU_UNIROBOT_SERIAL_PORT_HPP
#define SEU_UNIROBOT_SERIAL_PORT_HPP

#include <boost/asio.hpp>
#include "comm/comm.hpp"
#include "class_exception.hpp"

namespace comm
{
    class serial_port: public std::enable_shared_from_this<serial_port>
    {
    public:
        serial_port(boost::asio::io_service &io_service, const std::string &dev_name, const int &baudrate,
                ser_comm_callback cb=nullptr): serial_port_(io_service), timer_(io_service), cb_(std::move(cb))
        {
            is_open_ = false;
            rd_cplt_ = false;
            timeout_ = false;
            try
            {
                serial_port_.open(dev_name);
                serial_port_.set_option(boost::asio::serial_port::baud_rate(baudrate));
                serial_port_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
                serial_port_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
                serial_port_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
                serial_port_.set_option(boost::asio::serial_port::character_size(8));
            }
            catch(boost::system::system_error)
            {
                throw class_exception<serial_port>("open serial port failed");
            }
            is_open_ = true;
        }

        void write(const uint8_t *data, const int &size)
        {
            if(!is_open_) return;
            if(data == nullptr) return;
            auto self(shared_from_this());
            boost::asio::async_write(serial_port_, boost::asio::buffer(data, size),
                                     [this, self](boost::system::error_code ec, std::size_t length){});

        }

        void read(uint8_t *data, const int &size, const int &ms)
        {
            if(!is_open_) return;
            if(data == nullptr) return;
            rd_cplt_ = false;
            timeout_ = false;
            auto self(shared_from_this());
            boost::asio::async_read(serial_port_, boost::asio::buffer(data, size),
                [this, self](boost::system::error_code ec, std::size_t length)
                {
                    if(ec != 0) throw class_exception<serial_port>("there is something wrong with serial port");
                    rd_cplt_ = true;
                });
            timer_.expires_from_now(boost::posix_time::millisec(ms));
            timer_.async_wait(bind(&serial_port::time_out, this));
        }

        void start_read(const int &size)
        {
            auto self(shared_from_this());
            boost::asio::async_read(serial_port_, boost::asio::buffer(data_, size),
                [this, self, size](boost::system::error_code ec, std::size_t length)
                {
                    if(!ec)
                    {
                        if(cb_!= nullptr) cb_(data_, size);
                        start_read(size);
                    }
                    else throw class_exception<serial_port>("there is something wrong with serial port");
                });
        }

        void close()
        {
            serial_port_.close();
        }

        void time_out()
        {
            serial_port_.cancel();
            timeout_ = true;
        }

        bool timeout() const
        {
            return timeout_;
        }

        bool read_complete() const
        {
            return rd_cplt_;
        }

    private:
        boost::asio::serial_port serial_port_;
        boost::asio::deadline_timer timer_;
        bool is_open_, timeout_, rd_cplt_;
        ser_comm_callback cb_;
        char data_[256];
    };
}

#endif