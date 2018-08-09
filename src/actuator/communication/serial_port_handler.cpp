#include "serial_port_handler.hpp"

namespace communication 
{
    using namespace comm;
    const std::string serial_packet::header_ = "AB";
    
    serial_packet::serial_packet():body_size_(0)
    {
    }
    
    serial_packet::serial_packet(const serial_command &cmd)
    {
        std::memcpy(data_, header_.c_str(), header_length);
        body_size_ = cmd.size+2*sizeof(int);
        encode_size();
        std::memcpy(body(), (char*)(&cmd), body_size_);
    }
    
    bool serial_packet::decode_size()
    {
        int body_size;
        std::memcpy(&body_size, data_+header_length, size_length);
        if(body_size>max_body_length) return false;
        body_size_ = body_size;
        return true;
    }
        
    void serial_packet::encode_size()
    {
        std::memcpy(data_+header_length, (char*)(&body_size_), size_length);
        decode_size();
    }
    
    void serial_packet::clear()
    {
        std::memset(data_, 0, length());
        body_size_ = 0;
    }
    
    serial_port::serial_port(boost::asio::io_service &io_service, comm_callback cb)
        : serial_port_(io_service), cb_(std::move(cb)), timer_(io_service)
    {
        started_ = false;
        timer_.expires_at(boost::posix_time::pos_infin);
        get_config_info();
        serial_init();
    }
    
    void serial_port::write(const serial_packet::serial_command& cmd)
    {
        do_write(serial_packet(cmd));
    }
    
    void serial_port::close()
    {
        serial_port_.close();
    }
        
    void serial_port::start()
    {
        do_read_header(0);
    }
    
    void serial_port::time_out()
    {
        serial_port_.cancel();
        if(started_ && timer_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
            throw class_exception<serial_port>("serial port time out");
    }
    
    void serial_port::do_read_header(int idx)
    {
        if(idx==serial_packet::header_length)
            do_read_size();
        else
        {
            auto self(shared_from_this());
            timer_.expires_from_now(boost::posix_time::seconds(time_out_sec_));
            boost::asio::async_read(serial_port_, boost::asio::buffer(recv_pkt_.data()+idx, 1),
                                    [this, self, idx](boost::system::error_code ec, std::size_t length)
            {
                if(ec != 0) throw class_exception<serial_port>("there is something wrong with serial port");
                if(recv_pkt_.data()[idx] == serial_packet::header_.at(idx))
                    do_read_header(idx+1);
                else
                    do_read_header(0);
            });
            
            timer_.async_wait(std::bind(&serial_port::time_out, this));
        }
    }
    
    void serial_port::do_read_size()
    {
        auto self(shared_from_this());
        boost::asio::async_read(serial_port_, boost::asio::buffer(recv_pkt_.data()+serial_packet::header_length, serial_packet::size_length),
                                [this, self](boost::system::error_code ec, std::size_t length)
        {
            if(ec != 0) throw class_exception<serial_port>("there is something wrong with serial port");
            if(recv_pkt_.decode_size())
                do_read_body();
            else
                do_read_header(0);
        });
        timer_.expires_from_now(boost::posix_time::seconds(time_out_sec_));
        timer_.async_wait(std::bind(&serial_port::time_out, this));
    }
    
    void serial_port::do_read_body()
    {
        auto self(shared_from_this());
        boost::asio::async_read(serial_port_, boost::asio::buffer(recv_pkt_.body(), recv_pkt_.body_size()),
                                [this, self](boost::system::error_code ec, std::size_t length)
        {
            if(ec != 0) throw class_exception<serial_port>("there is something wrong with serial port");
            cb_(recv_pkt_.body(), recv_pkt_.body_size());
            recv_pkt_.clear();
            do_read_header(0);
        });
        timer_.expires_from_now(boost::posix_time::seconds(time_out_sec_));
        timer_.async_wait(std::bind(&serial_port::time_out, this));
        if(!started_) started_ = true;
    }
    
    void serial_port::do_write(const serial_packet &pkt)
    {
        auto self(shared_from_this());
        boost::asio::async_write(serial_port_, boost::asio::buffer(pkt.data(), pkt.length()),
                                [this, self](boost::system::error_code ec, std::size_t length){});
    }
    
    void serial_port::get_config_info()
    {
        dev_name_="/dev/ttyUSB0";
        baudrate_ = 9600;
        time_out_sec_ = 5;
    }
    
    void serial_port::serial_init()
    {
        try
        {
            serial_port_.open(dev_name_);
            serial_port_.set_option(boost::asio::serial_port::baud_rate(baudrate_));
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
    
    serial_port_handler::serial_port_handler()
    {
        serial_ = std::make_shared<serial_port>(io_service_, std::bind(&serial_port_handler::data_handler, this, std::placeholders::_1, std::placeholders::_2));
    }
    
    void serial_port_handler::data_handler(const char *data, const int size)
    {
        serial_packet::serial_command cmd;
        std::memcpy(&cmd, data, size);
        std::cout.write(cmd.data,cmd.size);
        std::cout<<std::endl;
    }
    
    void serial_port_handler::write(const serial_packet::serial_command &cmd)
    {
        serial_->write(cmd);
    }
    
    void serial_port_handler::close()
    {
        serial_->close();
        io_service_.stop();
    }
    
    void serial_port_handler::run()
    {
        serial_->start();
        io_service_.run();
    }
}