#ifndef TCP_SERVER_HANDLER_HPP
#define TCP_SERVER_HPP

#include "comm/tcp/tcp_server.hpp"
#include "thread.hpp"
#include "singleton.hpp"

namespace communication
{
    using namespace comm;
    
    class tcp_server_handler: public thread, public singleton<tcp_server_handler>
    {
    public:
        tcp_server_handler()
        {
            get_config_info();
            server_ = tcp_server(io_service_, tcp::endpoint(tcp::v4(), port_), &data_handler);
        }
        
        static void data_handler(const char *data, const int &size)
        {
            if(data == nullptr) return;
        }
        
        void write(const tcp_packet::tcp_cmd_type &type, const int size, const char *data)
        {
            int t_size = size;
            tcp_packet::tcp_command cmd;
            cmd.type = type;
            int i=0;
            while(t_size >= tcp_packet::max_cmd_data_length)
            {
                cmd.size = tcp_packet::max_cmd_data_length;
                std::memcpy(cmd.data, data+i*tcp_packet::max_cmd_data_length, tcp_packet::max_cmd_data_length);
                i++;
                t_size -= tcp_packet::max_cmd_data_length;
                server_.do_write(tcp_packet(cmd));
                usleep(10);
            }
            cmd.size = t_size;
            std::memcpy(cmd.data, data+i*tcp_packet::max_cmd_data_length, t_size);
            server_.do_write(tcp_packet(cmd));
        }
        
        void write(const tcp_packet::tcp_command &cmd)
        {
            server_.do_write(tcp_packet(cmd));
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
        void get_config_info()
        {
        }
        
        boost::asio::io_service io_service_;
        tcp_server server_;
        int port_;
    };
    
    #define TCP_SERVER tcp_server_handler::get_singleton()
}
#endif