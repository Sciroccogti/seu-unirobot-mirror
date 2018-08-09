#ifndef UDP_SERVER_HANDLER_HPP
#define UDP_SERVER_HANDLER_HPP

#include "comm/udp/udp_server.hpp"
#include "thread.hpp"

namespace communication
{
    using namespace comm;
    
    class udp_server_handler: public thread
    {
    public:
        udp_server_handler(const std::string &udp_addr, const short &port, comm_callback cb)
        {
            udp::endpoint endpoint(boost::asio::ip::address_v4::from_string(udp_addr), port);
            server_ = udp_server(io_service_, endpoint, std::move(cb));
        }
        
        void write(const void *data, const int &size)
        {
            server_.do_send((char*)(data), size);
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
    };
}

#endif
