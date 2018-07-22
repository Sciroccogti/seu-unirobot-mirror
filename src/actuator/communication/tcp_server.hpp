#ifndef TCP_SERVER_HPP
#define TCP_SERVER_HPP

#include <string>
#include <vector>
#include <boost/asio.hpp>
#include "comm.hpp"

namespace communication
{
    using namespace boost::asio::ip;
    using namespace std;
    
    class tcp_server: public comm
    {
    public:
        tcp_server(unsigned int port): port_(port)
        {
        }
        bool open()
        {
            if(is_open_) return false;
            tcp::endpoint endpoint(tcp::v4(), port_);
            acceptor_ = 
        }
        bool close();
        bool write(const string &data);
        bool read(string &data, int &size);
        
    private:
        void accept()
        {
        }
    private:
        tcp::socket socket_;
        tcp::acceptor acceptor_;
        unsigned int port_;
    };
}
#endif