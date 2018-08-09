#ifndef SEU_UNIROBOT_SERIAL_PORT_HPP
#define SEU_UNIROBOT_SERIAL_PORT_HPP

#include <boost/asio.hpp>
#include "class_exception.hpp"

namespace comm
{
    class serial_port: public std::enable_shared_from_this<serial_port>
    {
    public:
        serial_port(boost::asio::io_service &io_service, const std::string &dev_name="", const int &baudrate=115200, const int &recv_timeout_ms=5)
        : serial_port_(io_service), timer_(io_service)
        {
            recv_timeout_ms_ = recv_timeout_ms;
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
        }

        void write(const uint8_t *data, const unsigned int &size)
        {
            auto self(shared_from_this());
            boost::asio::async_write(serial_port_, boost::asio::buffer(data, size),
                                    [this, self](boost::system::error_code ec, std::size_t length){});
        }

        void read(uint8_t *data, const unsigned int &size)
        {
            rc_ = false;
            time_out_ = false;
            auto self(shared_from_this());
            boost::asio::async_read(serial_port_, boost::asio::buffer(data, size),
                                    [this, self](boost::system::error_code ec, std::size_t length)
            {
                if(ec != 0) throw class_exception<serial_port>("there is something wrong with serial port");
                rc_ = true;
            });
            timer_.expires_from_now(boost::posix_time::seconds(recv_timeout_ms_));
            timer_.async_wait(std::bind(&serial_port::time_out, this));
        }
        
        void time_out()
        {
            if(timer_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
            {
                time_out_ = true;
                serial_port_.cancel();
            }
        }
        
        void close()
        {
            serial_port_.close();
        }

        bool read_complete() const
        {
            return rc_;
        }

        bool timeout() const
        {
            return time_out_;
        }

    private:
        int recv_timeout_ms_;
        bool rc_;
        bool time_out_;
        boost::asio::serial_port serial_port_;
        boost::asio::deadline_timer timer_;
    };
}

#endif