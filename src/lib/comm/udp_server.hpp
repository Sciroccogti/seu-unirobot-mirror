#ifndef UDP_SERVER_HPP
#define UDP_SERVER_HPP

#include <string>
#include <functional>
#include <boost/asio.hpp>
#include "comm.hpp"

namespace comm
{
    
    using boost::asio::ip::udp;

    class udp_server
    {
    public:
        enum {max_data_length = 1024};
        udp_server(boost::asio::io_service &io_service, const udp::endpoint &endpoint, const int &size, const std::string &header, const int &header_size, net_comm_callback cb=nullptr)
            : socket_(io_service,endpoint), cb_(std::move(cb)), header_(header), header_length_(header_size), size_(size)
        {
            do_read_header(0);
        }

        void do_send(const char *data, const int &size)
        {
             socket_.async_send_to(boost::asio::buffer(data, size), sender_endpoint_,
                [this](boost::system::error_code /*ec*/, std::size_t /*bytes_sent*/){});
        }
        
        void close()
        {
            socket_.close();
        }

    private:
        void do_read_header(int idx)
        {
            if(idx == header_length_) do_receive();
            socket_.async_receive_from(boost::asio::buffer(data_, idx), sender_endpoint_,
                [this, idx](boost::system::error_code ec, std::size_t)
                {
                if (!ec)
                {
                    if(data_[idx] == header_.at(idx)) do_read_header(idx+1);
                    else do_read_header(0);
                }
            });
        }
        
        void do_receive()
        {
            socket_.async_receive_from(boost::asio::buffer(data_+header_length_, size_-header_length_), sender_endpoint_,
                [this](boost::system::error_code ec, std::size_t bytes_recvd)
                {
                if (!ec && bytes_recvd > 0)
                {
                    cb_(data_, size_, 0);
                }
                do_receive();
            });
        }
        
        std::string header_;
        int header_length_;
        int size_;
        udp::socket socket_;
        udp::endpoint sender_endpoint_;
        char data_[max_data_length];
        net_comm_callback cb_;
    };
}

#endif