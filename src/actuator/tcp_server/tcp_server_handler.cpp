#include "tcp_server_handler.hpp"
#include "configuration.hpp"
#include "robot/humanoid.hpp"

using namespace comm;
using namespace std;
using namespace robot;

boost::asio::io_service tcp_io_service;

tcp_server_handler::tcp_server_handler()
    : server_(tcp_io_service, tcp::endpoint(tcp::v4(), CONF.get_config_value<int>("net.tcp.port")),
            bind(&tcp_server_handler::data_handler, this, placeholders::_1, placeholders::_2, placeholders::_3))
{
    r_data_.id = 0;
    r_data_.size = 0;
}

void tcp_server_handler::start()
{
    td_ = thread(bind(&tcp_server_handler::run, this));
}

void tcp_server_handler::data_handler(const char *data, const int &size, const int &type)
{
    //std::cout<<"new data, size: "<<size<<" type: "<<type<<std::endl;
    int float_size = sizeof(float);
    int int_size = sizeof(int);
    int type_size = sizeof(tcp_packet::remote_data_type);
    
    if(data == nullptr) return;
    std::lock_guard<std::mutex> lk(tcp_mutex_);
    switch(type)
    {
        case tcp_packet::JOINT_DATA:
        {
            if(size%sizeof(robot_joint_deg) == 0)
            {
                robot_joint_deg jd;
                for(int i=0;i<ROBOT.get_joint_map().size();i++)
                {
                    memcpy(&jd, data+i*sizeof(robot_joint_deg), sizeof(robot_joint_deg));
                    cout<<jd.id<<'\t'<<jd.deg<<endl;
                }
            }
            break;
        }
        case tcp_packet::REMOTE_DATA:
        {
            memcpy(&(r_data_.type), data, type_size);
            //std::cout<<"recv data: "<<(int)r_data_.type<<'\n';
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
                std::cout<<"recv act data \n";
                int blksz = sizeof(int)+ROBOT.get_joint_map().size()* sizeof(robot_joint_deg);
                if((size-type_size)%blksz == 0)
                {
                    r_data_.data.clear();
                    r_data_.data.assign(data+type_size, size-type_size);
                }
            }
            r_data_.size = size-type_size;
            r_data_.id++;
            break;
        }
        default:
            break;
    }
}

void tcp_server_handler::write(const comm::tcp_packet::tcp_cmd_type &type, const int &size, const char *data)
{
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

void tcp_server_handler::write(const comm::tcp_packet::tcp_command &cmd)
{
    server_.do_write(comm::tcp_packet(cmd));
}

void tcp_server_handler::run()
{
    tcp_io_service.run();
}

void tcp_server_handler::close()
{
    tcp_io_service.stop();
    std::cout<<"\033[32mtcp_server closed!\033[0m\n";
}

tcp_server_handler::~tcp_server_handler()
{
    if(td_.joinable()) td_.join();
}
