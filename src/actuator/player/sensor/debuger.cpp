#include "debuger.hpp"
#include "configuration.hpp"
#include "robot/humanoid.hpp"

using namespace comm;
using namespace std;
using namespace robot;

boost::asio::io_service tcp_io_service;

debuger::debuger(const sub_ptr& s): sensor("debuger"),
    server_(tcp_io_service, tcp::endpoint(tcp::v4(), CONF.get_config_value<int>("net.tcp.port")),
            bind(&debuger::data_handler, this, placeholders::_1, placeholders::_2, placeholders::_3))
{
    attach(s);
    r_data_.size = 0;
    r_data_.type = tcp_packet::NONE_DATA;
}

bool debuger::open()
{
    is_alive_ = true;
    return true;
}

bool debuger::start()
{
    this->open();
    td_ = thread(bind(&debuger::run, this));
    return true;
}

void debuger::run()
{
    tcp_io_service.run();
}

void debuger::stop()
{
    server_.close();
    tcp_io_service.stop();
    is_alive_ = false;
}

void debuger::data_handler(const char* data, const int& size, const int& type)
{
    int float_size = sizeof(float);
    int int_size = sizeof(int);
    int type_size = sizeof(tcp_packet::remote_data_type);
    
    if(data == nullptr) return;
    switch(type)
    {
        case tcp_packet::JOINT_DATA:
        {
            if(size%sizeof(robot_joint_deg) == 0)
            {
                robot_joint_deg jd;
                for(int i=0;i<ROBOT.get_joint_map().size();i++)
                    memcpy(&jd, data+i*sizeof(robot_joint_deg), sizeof(robot_joint_deg));
            }
            break;
        }
        case tcp_packet::REMOTE_DATA:
        {
            memcpy(&(r_data_.type), data, type_size);
            if(r_data_.type == tcp_packet::WALK_DATA)
            {
                if(size == 4*float_size+type_size)
                {
                    r_data_.data.clear();
                    r_data_.data.assign(data+type_size, size-type_size);
                }
            }
            else if(r_data_.type == tcp_packet::ACT_DATA)
            {
                int blksz = sizeof(int)+ROBOT.get_joint_map().size()* (int_size+float_size);
                if((size-type_size)%blksz == 0)
                {
                    r_data_.data.clear();
                    r_data_.data.assign(data+type_size, size-type_size);
                }
            }
            r_data_.size = size-type_size;
            break;
        }
        default:
            break;
    }
    notify();
}

void debuger::write(const tcp_packet::tcp_command& cmd)
{
    if(!is_alive_) return;
    server_.do_write(comm::tcp_packet(cmd));
}

void debuger::write(const tcp_packet::tcp_cmd_type& type, const int& size, const char* data)
{
    if(!is_alive_) return;
    int t_size = size;
    comm::tcp_packet::tcp_command cmd;
    cmd.type = type;
    int i=0;
    while(t_size >= comm::tcp_packet::max_cmd_data_length)
    {
        cmd.size = comm::tcp_packet::max_cmd_data_length;
        std::memcpy(cmd.data, data+i*comm::tcp_packet::max_cmd_data_length, comm::tcp_packet::max_cmd_data_length);
        i++;
        t_size -= comm::tcp_packet::max_cmd_data_length;
        server_.do_write(comm::tcp_packet(cmd));
        usleep(10);
    }
    cmd.size = t_size;
    std::memcpy(cmd.data, data+i*comm::tcp_packet::max_cmd_data_length, t_size);
    server_.do_write(comm::tcp_packet(cmd));
}

debuger::~debuger()
{
    if(td_.joinable()) td_.join();
}


