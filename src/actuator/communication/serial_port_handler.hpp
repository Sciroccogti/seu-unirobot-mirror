#ifndef SERIAL_PORT_HANDLER_HPP
#define SERIAL_PORT_HANDLER_HPP

#include <string>
#include <boost/asio.hpp>
#include "thread.hpp"
#include "comm/comm.hpp"
#include "sensor/gyro.hpp"
#include "class_exception.hpp"

namespace communication
{
    class serial_packet
    {
    public:
        static const std::string header_;
        enum {header_length=2};
        enum {size_length=4};
        enum {max_body_length = 1024};
        enum serial_cmd_type
        {
            DATA_HEAD = 1,
            DATA_BODY = 2
        };
        
        struct serial_command
        {
            serial_cmd_type type;
            int size;
            char data[max_body_length-2*sizeof(int)];
        };
        
        serial_packet();
        serial_packet(const serial_command &cmd);
        bool decode_size();
        void encode_size();
        void clear();
        int body_size() const { return body_size_; }
        int length() const { return header_length+size_length+body_size_; }
        char *data() { return data_; }
        const char *data() const { return data_; }
        char *body() { return data_+header_length+size_length; }
        const char *body() const { return data_+header_length+size_length; }
    private:
        char data_[header_length+size_length+max_body_length];
        int body_size_;
    };

    class serial_port: public std::enable_shared_from_this<serial_port>
    {
    public:
        serial_port(boost::asio::io_service &io_service, comm::comm_callback cb);
        void write(const serial_packet::serial_command &cmd);
        void close();
        void start();
        void time_out();
        
    private:
        void do_read_header(int idx);      
        void do_read_size();
        void do_read_body();
        void do_write(const serial_packet &pkt);
        void get_config_info();
        void serial_init();
        
        bool started_;
        int time_out_sec_;
        std::string dev_name_;
        int baudrate_;
        boost::asio::serial_port serial_port_;
        serial_packet recv_pkt_;
        comm::comm_callback cb_;
        boost::asio::deadline_timer timer_;
    };
    
    class serial_port_handler
    {
    public:
        serial_port_handler();
        void data_handler(const char *data, const int size);
        void write(const serial_packet::serial_command &cmd);
        void close();
        void run();
    private:
        boost::asio::io_service io_service_;
        std::shared_ptr<serial_port> serial_;
    };
}

#endif