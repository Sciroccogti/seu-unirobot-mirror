#include "server.hpp"
#include "configuration.hpp"
#include "robot/humanoid.hpp"
#include <string>

using namespace std;
using namespace robot;
using boost::asio::ip::tcp;

void tcp_pool::join(tcp_session_ptr session)
{
    std::cout<<"connection from: "<<session->info()<<" joined\n";
    sessions_.insert(session);
}

void tcp_pool::leave(tcp_session_ptr session)
{
    std::cout<<"connection from: "<<session->info()<<" leaved\n";
    sessions_.erase(session);
}

void tcp_pool::deliver(const tcp_command& cmd)
{
    for(auto &session : sessions_)
    {
        if(session->check_type(cmd.type))
            session->deliver(cmd);
    }
}

void tcp_pool::close()
{
    for (auto &session : sessions_)
        session->stop();
}

tcp_session::tcp_session(tcp::socket sock, tcp_pool& pool, tcp_callback ncb)
    : socket_(std::move(sock)), pool_(pool), tcb_(std::move(ncb))
{
    info_.clear();
    info_.append(socket_.remote_endpoint().address().to_string()+":"+to_string(socket_.remote_endpoint().port()));
    recv_cmd_.type = NONE_DATA;
    memset(buff_, 0, MAX_CMD_LEN);
}

void tcp_session::start()
{
    pool_.join(shared_from_this());
    this->read_head();
}

void tcp_session::read_head()
{
    auto self(shared_from_this());
    boost::asio::async_read(socket_, boost::asio::buffer(buff_, data_offset),
                            [this, self](boost::system::error_code ec, std::size_t length)
    {
        if (!ec)
        {
            memcpy(&recv_type_, buff_, tcp_type_size);
            memcpy(&recv_end_, buff_+tcp_type_size, tcp_end_size);
            memcpy(&recv_size_, buff_+tcp_type_size+tcp_end_size, tcp_size_size);
            read_data();
        }
        else
        {
            pool_.leave(shared_from_this());
        }
    });
}

void tcp_session::read_data()
{
    auto self(shared_from_this());
    boost::asio::async_read(socket_, boost::asio::buffer(buff_+data_offset, recv_size_),
                            [this, self](boost::system::error_code ec, std::size_t length)
    {
        if (!ec)
        {
            if(recv_cmd_.type != recv_type_)
            {
                recv_cmd_.type = recv_type_;
                recv_cmd_.end = recv_end_;
                recv_cmd_.size = recv_size_;
                recv_cmd_.data.clear();
                recv_cmd_.data.assign((char*)(buff_+data_offset), recv_size_);
            }
            else
            {
                if(!recv_cmd_.end)
                {
                    recv_cmd_.end = recv_end_;
                    recv_cmd_.size+=recv_size_;
                    recv_cmd_.data.append((char*)(buff_+data_offset), recv_size_);
                }
                else
                {
                    recv_cmd_.type = recv_type_;
                    recv_cmd_.end = recv_end_;
                    recv_cmd_.size = recv_size_;
                    recv_cmd_.data.clear();
                    recv_cmd_.data.assign((char*)(buff_+data_offset), recv_size_);
                }
            }
            if(recv_end_)
            {
                if(recv_cmd_.type == REG_DATA)
                {
                    tcp_cmd_type t;
                    tcp_data_dir d;
                    std::memcpy(&t, recv_cmd_.data.c_str(), tcp_type_size);
                    std::memcpy(&d, recv_cmd_.data.c_str()+tcp_type_size, tcp_dir_size);
                    td_map_[t] = d;
                }
                else
                {
                    tcb_(recv_cmd_);
                }
            }
            read_head();
        }
        else
        {
            pool_.leave(shared_from_this());
        }
    });
}


void tcp_session::stop()
{
    socket_.close();
}

bool tcp_session::check_type(const tcp_cmd_type& t)
{
    for(auto &c:td_map_)
        if(c.first == t && (c.second == DIR_BOTH || c.second == DIR_APPLY))
            return true;
    return false;
}

void tcp_session::deliver(const tcp_command& cmd)
{
    unsigned int data_size = cmd.size;
    unsigned int total_size = data_size+data_offset;
    char buf[MAX_CMD_LEN];
    unsigned int offset=0;
    memcpy(buf+offset, &(cmd.type),tcp_type_size);
    offset+=tcp_type_size;
    memcpy(buf+offset, &(cmd.end), tcp_end_size);
    offset+=tcp_end_size;
    memcpy(buf+offset, &data_size, tcp_size_size);
    offset+=tcp_size_size;
    memcpy(buf+offset, cmd.data.c_str(), cmd.data.size());
    
    auto self(shared_from_this());
    boost::asio::async_write(socket_, boost::asio::buffer(buf, total_size),
                            [this, self](boost::system::error_code ec, std::size_t length)
    {
        if (ec) pool_.leave(shared_from_this());
    });
}
    
    
boost::asio::io_service tcp_service;

tcp_server::tcp_server()
    :sensor("server"), socket_(tcp_service), 
    acceptor_(tcp_service, tcp::endpoint(tcp::v4(), CONF->get_config_value<int>("net.tcp.port")))
{
    r_data_.type = NON_DATA;
    r_data_.size = 0;
}

void tcp_server::accept()
{
    acceptor_.async_accept(socket_, [this](boost::system::error_code ec)
    {
        if (!ec)
        {
            std::make_shared<tcp_session>(std::move(socket_), pool_, 
                bind(&tcp_server::data_handler, this, placeholders::_1))->start();
        }
        accept();
    });
}

void tcp_server::data_handler(const tcp_command cmd)
{
    switch(cmd.type)
    {
        case JOINT_DATA:
        {
            if(cmd.size%sizeof(robot_joint_deg) == 0)
            {
                robot_joint_deg jd;
                for(int i=0;i<ROBOT->get_joint_map().size();i++)
                    memcpy(&jd, cmd.data.c_str()+i*sizeof(robot_joint_deg), sizeof(robot_joint_deg));
            }
            break;
        }
        case REMOTE_DATA:
        {
            memcpy(&(r_data_.type), cmd.data.c_str(), rmt_type_size);
            r_data_.data.clear();
            r_data_.data.assign((char*)(cmd.data.c_str()+rmt_type_size), cmd.size-rmt_type_size);
            r_data_.size = cmd.size-rmt_type_size;
            break;
        }
        default:
            break;
    }
    notify(SENSOR_SERVER);
}

void tcp_server::write(const tcp_command& cmd)
{
    if(!sensor::is_alive_) return;
    unsigned int t_size = cmd.size;
    unsigned max_data_size = MAX_CMD_LEN - tcp_size_size - tcp_type_size - tcp_end_size;
    int i=0;
    tcp_command temp;
    temp.type = cmd.type;
    while(t_size > max_data_size)
    {
        temp.end = 0;
        temp.size = max_data_size;
        temp.data.assign((char*)(cmd.data.c_str()+i*max_data_size), max_data_size);
        pool_.deliver(temp);
        t_size -= max_data_size;
        i++;
        usleep(10);
    }
    temp.size = t_size;
    temp.end = 1;
    temp.data.assign((char*)(cmd.data.c_str()+i*max_data_size), t_size);
    pool_.deliver(temp);
}

bool tcp_server::start()
{
    is_alive_ = true;
    td_ = std::move(std::thread([this]()
    {
        accept();
        tcp_service.run();
    }));
    return true;
}

void tcp_server::stop()
{
    pool_.close();
    tcp_service.stop();
    is_alive_ = false;
}

tcp_server::~tcp_server()
{
    if(td_.joinable()) td_.join();
}
