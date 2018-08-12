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
        udp_server(boost::asio::io_service &io_service, const udp::endpoint &endpoint, tcp_comm_callback cb)
            : socket_(io_service,endpoint), cb_(std::move(cb))
        {
            do_receive();
        }

        void do_receive()
        {
            socket_.async_receive_from(boost::asio::buffer(data_, max_data_length), sender_endpoint_,
                [this](boost::system::error_code ec, std::size_t bytes_recvd)
                {
                if (!ec && bytes_recvd > 0)
                {
                    cb_(data_, bytes_recvd);
                }
                do_receive();
            });
        }
        
        void do_send(const char *data, const int &size)
        {
             socket_.async_send_to(boost::asio::buffer(data, size), sender_endpoint_,
                [this](boost::system::error_code /*ec*/, std::size_t /*bytes_sent*/){});
        }

    private:
        udp::socket socket_;
        udp::endpoint sender_endpoint_;
        char data_[max_data_length];
        tcp_comm_callback cb_;
    };
}

#endif