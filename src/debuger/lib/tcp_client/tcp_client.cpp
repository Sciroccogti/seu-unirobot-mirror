#include "tcp_client.hpp"
#include "logger.hpp"
#include <string>

using boost::asio::ip::tcp;

boost::asio::io_service tcp_service;

tcp_client::tcp_client(const std::string& addr, const int& port, tcp_callback tcb)
    :port_(port), addr_(addr), socket_(tcp_service), tcb_(tcb)
{
    is_connect_ = false;
}

void tcp_client::write(const tcp_command& cmd)
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
        deliver(temp);
        t_size -= max_data_size;
        i++;
        usleep(10);
    }
    temp.size = t_size;
    temp.end = 1;
    temp.data.assign((char*)(cmd.data.c_str()+i*max_data_size), t_size);
    deliver(temp);
}

void tcp_client::regist(const tcp_cmd_type &type, const tcp_data_dir &dir)
{
    tcp_command cmd;
    cmd.type = REG_DATA;
    cmd.size = tcp_type_size+tcp_dir_size;
    cmd.data.clear();
    cmd.data.append((char*)&type, tcp_type_size);
    cmd.data.append((char*)&dir, tcp_dir_size);
    this->write(cmd);
}

void tcp_client::deliver(const tcp_command& cmd)
{
    if(!is_connect_) return;
    boost::system::error_code ec;
    unsigned int data_size = cmd.size;
    unsigned int total_size = data_size + tcp_size_size + tcp_type_size + tcp_full_size;
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
    if(ec) is_connect_ = true;
    delete []buf;
}

void tcp_client::start()
{
    is_alive_ = true;
    td_ = std::thread([this]()
    {
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
            if(!is_connect_)
            {
                tcp::endpoint ep(boost::asio::ip::address::from_string(addr_), port_);
                socket_.connect(ep, ec);
                if(ec)
                {
                    sleep(1);
                    continue;
                }
                is_connect_ = true;
            }
            if(is_connect_)
            {
                while(is_alive_)
                {
                    /*
                    ablen = socket_.available(ec);
                    if(ec)
                    {
                        LOG(LOG_WARN)<<"sever closed\n";
                        is_connect_ = false;
                        break;
                    }
                    if(!ablen) continue;*/
                    memset(buff, 0, MAX_CMD_LEN);
                    socket_.read_some(boost::asio::buffer(buff), ec);
                    if(ec)
                    {
                        LOG(LOG_WARN)<<"sever closed\n";
                        is_connect_ = false;
                        socket_.close();
                        break;
                    }
                    else
                    {
                        offset=0;
                        memcpy(&recv_type, buff+offset, tcp_type_size);
                        offset+=tcp_type_size;
                        memcpy(&recv_end, buff+offset, 1);
                        offset+=1;
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
                            if(tcb_ != nullptr) tcb_(recv_cmd);
                        }
                    }
                }
            }
        }
        socket_.close();
    });
}

void tcp_client::stop()
{
    is_alive_ = false;
}

tcp_client::~tcp_client()
{
    if(td_.joinable()) td_.join();
}

