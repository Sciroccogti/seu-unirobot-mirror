#ifndef SEU_UNIROBOT_ACTUATOR_DEBUGER_HPP
#define SEU_UNIROBOT_ACTUATOR_DEBUGER_HPP

#include <boost/asio.hpp>
#include <vector>
#include <list>
#include <set>
#include <thread>
#include <mutex>
#include "tcp.hpp"
#include "sensor/sensor.hpp"
#include "timer.hpp"

class tcp_session;
typedef std::shared_ptr<tcp_session> tcp_session_ptr;

class tcp_pool
{
public:
    void join(tcp_session_ptr session);
    void leave(tcp_session_ptr session);
    void close();
    void deliver(const tcp_command &cmd);
private:
    std::set<tcp_session_ptr> sessions_;
};

class tcp_session: public std::enable_shared_from_this<tcp_session>
{
public:
    tcp_session(boost::asio::ip::tcp::socket sock, tcp_pool &pool, tcp_callback ncb);
    void start();
    void stop();
    void deliver(const tcp_command& cmd);
    bool check_type(const tcp_cmd_type &t);
    std::string info() const
    {
        return info_;
    }
private:
    std::map<tcp_cmd_type, tcp_data_dir> td_map_;
    boost::asio::ip::tcp::socket socket_;
    bool is_alive_;
    tcp_pool &pool_;
    tcp_callback tcb_;
    std::string info_;
};

class tcp_server: public sensor, public timer
{
public:
    tcp_server(const sub_ptr& s);
    ~tcp_server();
    bool start();
    void stop();
    void write(const tcp_command &cmd);
    remote_data r_data() const
    {
        return r_data_;
    }
    
private:
    void run();
    void data_handler(const tcp_command cmd);
    std::vector<std::thread> session_threads_;
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;
    std::thread td_;
    tcp_pool pool_;
    remote_data r_data_;
    bool is_alive_;
    int port_;
};

#endif