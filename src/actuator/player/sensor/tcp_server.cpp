#include "tcp_server.hpp"
#include "configuration.hpp"
#include "robot/humanoid.hpp"

using namespace std;
using namespace robot;
using boost::asio::ip::tcp;

void tcp_pool::join(tcp_session_ptr session)
{
    LOG<<LOG_INFO<<"connection from: "<<session->info()<<" joined"<<"\n";
    sessions_.insert(session);
}

void tcp_pool::leave(tcp_session_ptr session)
{
    LOG<<LOG_INFO<<"connection from: "<<session->info()<<" leaved"<<"\n";
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
    is_alive_ = false;
}

void tcp_session::start()
{
    pool_.join(shared_from_this());
    is_alive_ = true;
    boost::system::error_code ec;
    char buff[MAX_CMD_LEN];
    tcp_command recv_cmd;
    tcp_cmd_type recv_type;
    unsigned char recv_end;
    unsigned int recv_size;
    recv_cmd.type = NONE_DATA;
    unsigned int offset=0;
    unsigned int ablen=0;
    while(is_alive_)
    {
        
        ablen = socket_.available(ec);
        if(ec)
        {
            pool_.leave(shared_from_this());
            break;
        }
        if(!ablen) continue;
        
        //if(!socket_.available()) continue;
        memset(buff, 0, MAX_CMD_LEN);
        socket_.read_some(boost::asio::buffer(buff), ec);
        if(ec)
        {
            pool_.leave(shared_from_this());
            break;
        }
        else
        {
            offset=0;
            memcpy(&recv_type, buff+offset, tcp_type_size);
            offset+=tcp_type_size;
            memcpy(&recv_end, buff+offset, tcp_full_size);
            offset+=tcp_full_size;
            memcpy(&recv_size, buff+offset, tcp_size_size);
            offset+=tcp_size_size;
            if(recv_cmd.type != recv_type)
            {
                recv_cmd.type = recv_type;
                recv_cmd.end = recv_end;
                recv_cmd.size = recv_size;
                recv_cmd.data.clear();
                recv_cmd.data.assign((char*)(buff+offset), recv_size);
            }
            else
            {
                if(!recv_cmd.end)
                {
                    recv_cmd.end = recv_end;
                    recv_cmd.size+=recv_size;
                    recv_cmd.data.append((char*)(buff+offset), recv_size);
                }
                else
                {
                    recv_cmd.type = recv_type;
                    recv_cmd.end = recv_end;
                    recv_cmd.size = recv_size;
                    recv_cmd.data.clear();
                    recv_cmd.data.assign((char*)(buff+offset), recv_size);
                }
            }
            if(recv_end)
            {
                if(recv_cmd.type == REG_DATA)
                {
                    tcp_cmd_type t;
                    tcp_data_dir d;
                    std::memcpy(&t, recv_cmd.data.c_str(), tcp_type_size);
                    std::memcpy(&d, recv_cmd.data.c_str()+tcp_type_size, tcp_dir_size);
                    td_map_[t] = d;
                }
                else
                {
                    tcb_(recv_cmd);
                }
            }
        }
    }
    socket_.close();
}

void tcp_session::stop()
{
    is_alive_ = false;
}

bool tcp_session::check_type(const tcp_cmd_type& t)
{
    if(t == BEAT_DATA) return true;
    for(auto c:td_map_)
        if(c.first == t && (c.second == DIR_BOTH || c.second == DIR_APPLY))
            return true;
    return false;
}

void tcp_session::deliver(const tcp_command& cmd)
{
    boost::system::error_code ec;
    if(!is_alive_) return;
    unsigned int data_size = cmd.size;
    unsigned int total_size = data_size+tcp_size_size+tcp_type_size+tcp_full_size;
    char *buf = new char[total_size];
    unsigned int offset=0;
    memcpy(buf+offset, &(cmd.type),tcp_type_size);
    offset+=tcp_type_size;
    memcpy(buf+offset, &(cmd.end), tcp_full_size);
    offset+=tcp_full_size;
    memcpy(buf+offset, &data_size, tcp_size_size);
    offset+=tcp_size_size;
    memcpy(buf+offset, cmd.data.c_str(), cmd.data.size());
    socket_.write_some(boost::asio::buffer(buf, total_size), ec);
    if(ec)
    {
        pool_.leave(shared_from_this());
        stop();
    }
    delete []buf;
}
    
boost::asio::io_service tcp_service;

tcp_server::tcp_server(const sub_ptr& s)
    :sensor("server"), timer(1000), port_(CONF.get_config_value<int>("net.tcp.port")),
    socket_(tcp_service), acceptor_(tcp_service, tcp::endpoint(tcp::v4(), CONF.get_config_value<int>("net.tcp.port")))
{
    attach(s);
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
                for(int i=0;i<ROBOT.get_joint_map().size();i++)
                    memcpy(&jd, cmd.data.c_str()+i*sizeof(robot_joint_deg), sizeof(robot_joint_deg));
            }
            break;
        }
        case REMOTE_DATA:
        {
            memcpy(&(r_data_.type), cmd.data.c_str(), rmt_type_size);
            if(r_data_.type == WALK_DATA)
            {
                if(cmd.size == 4*float_size+rmt_type_size)
                {
                    r_data_.data.clear();
                    r_data_.data.assign((char*)(cmd.data.c_str()+rmt_type_size), cmd.size-rmt_type_size);
                }
            }
            else if(r_data_.type == ACT_DATA)
            {
                int blksz = int_size+ROBOT.get_joint_map().size()*(int_size+float_size);
                if((cmd.size-rmt_type_size)%blksz == 0)
                {
                    r_data_.data.clear();
                    r_data_.data.assign((char*)(cmd.data.c_str()+rmt_type_size), cmd.size-rmt_type_size);
                }
            }
            r_data_.size = cmd.size;
            break;
        }
        default:
            break;
    }
    notify();
}

void tcp_server::write(const tcp_command& cmd)
{
    if(!is_alive_) return;
    unsigned int t_size = cmd.size;
    unsigned max_data_size = MAX_CMD_LEN - tcp_size_size - tcp_type_size - tcp_full_size;
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
        boost::system::error_code ec;
        while(is_alive_)
        {
            acceptor_.accept(socket_);
            if(!is_alive_) break;
            session_threads_.emplace_back(std::thread([this]()
            {
                std::make_shared<tcp_session>(std::move(socket_), pool_, std::bind(&tcp_server::data_handler, this, std::placeholders::_1))->start();
            }));
            usleep(10000);
        }
        acceptor_.close();
    }));
    start_timer();
    return true;
}

void tcp_server::run()
{
    if(is_alive_)
    {
        tcp_command cmd;
        cmd.type = BEAT_DATA;
        cmd.size = 0;
        this->write(cmd);
    }
}

void tcp_server::stop()
{
    is_alive_ = false;
    boost::asio::io_service tio;
    tcp::socket ts(tio);
    tcp::endpoint ep(boost::asio::ip::address::from_string("127.0.0.1"), port_);
    ts.connect(ep);
    ts.close();
    pool_.close();
    delete_timer();
}

tcp_server::~tcp_server()
{
    for(auto &st:session_threads_)
        if(st.joinable()) st.join();
    if(td_.joinable()) td_.join();
}
