#ifndef UDP_SERVER_HPP
#define UDP_SERVER_HPP

#include <string>
#include <boost/asio.hpp>
#include "udp_packet.hpp"
#include "thread.hpp"

namespace comm
{
    
    using boost::asio::ip::udp;

    class udp_server
    {
    public:
        udp_server(boost::asio::io_service &io_service, const udp::endpoint &endpoint)
            : socket_(io_service,endpoint)
        {
            do_receive();
        }

        void do_receive()
        {
            socket_.async_receive_from(boost::asio::buffer(recv_pkt_.data(), udp_packet::max_data_length), sender_endpoint_,
                [this](boost::system::error_code ec, std::size_t bytes_recvd)
                {
                if (!ec && bytes_recvd > 0)
                {
                    do_send(bytes_recvd);
                }
                else
                {
                    do_receive();
                }
            });
        }
        
        void do_send(const udp_packet &pkt)
        {
             socket_.async_send_to(boost::asio::buffer(pkt.data(), pkt.length()), sender_endpoint_,
                [this](boost::system::error_code /*ec*/, std::size_t /*bytes_sent*/){});
        }

        void do_send(std::size_t length)
        {
            socket_.async_send_to(boost::asio::buffer(recv_pkt_.data(), length), sender_endpoint_,
                [this](boost::system::error_code /*ec*/, std::size_t /*bytes_sent*/)
                {
                do_receive();
                });
        }

    private:
        udp::socket socket_;
        udp::endpoint sender_endpoint_;
        udp_packet recv_pkt_;
    };
    
    class udp_server_thread: public thread
    {
    public:
        udp_server_thread(const std::string &udp_addr, const short &port, const std::string &name): name_(name)
        {
            udp::endpoint endpoint(boost::asio::ip::address_v4::from_string(udp_addr), port);
            server_ = udp_server(io_service_, endpoint);
        }
        
        void write(const void *data, const int &size)
        {
            server_.do_send(udp_packet(reinterpret_cast<char*>(data), size));
        }
        
        void close()
        {
            io_service_.stop();
        }
    protected:
        void run()
        {
            io_service_.run();
        }
    private:
        boost::asio::io_service io_service_;
        udp_server server_;
        std::string name_;
    };
}

#endif