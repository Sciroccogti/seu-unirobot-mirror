#include "tcp_server_handler.hpp"
#include "configuration.hpp"
#include "robot/humanoid.hpp"

using namespace comm;
using namespace std;
using namespace robot;

boost::asio::io_service tcp_io_service;

tcp_server_handler::tcp_server_handler()
    : server_(tcp_io_service, tcp::endpoint(tcp::v4(), CONF.get_config_value<int>("tcp.port")),
            std::bind(&tcp_server_handler::data_handler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3))
{
}

void tcp_server_handler::data_handler(const char *data, const int &size, const int &type)
{
    //std::cout<<"new data, size: "<<size<<" type: "<<type<<std::endl;
    int float_size = sizeof(float);
    int int_size = sizeof(int);
    if(data == nullptr) return;
    switch(type)
    {
        case tcp_packet::JOINT_OFFSET:
        {
            if(size == ROBOT.get_joint_map().size()*sizeof(robot_joint_deg))
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
        case tcp_packet::POS_DATA:
        {
            int blksz = sizeof(int)+ROBOT.get_joint_map().size()* sizeof(robot_joint_deg);
            if(size%blksz == 0)
            {
                for(int k=0;k<size/blksz;k++)
                {
                    int act_t;
                    memcpy(&act_t, data+k*blksz, int_size);
                    robot_joint_deg jd;
                    cout<<"act_time: "<<act_t<<endl;
                    for(int i=0;i<ROBOT.get_joint_map().size();i++)
                    {
                        memcpy(&jd, data+k*blksz+int_size+i*sizeof(robot_joint_deg), sizeof(robot_joint_deg));
                        cout<<jd.id<<'\t'<<jd.deg<<endl;
                    }
                }
            }
            break;
        }
        case tcp_packet::WALK_DATA:
        {
            if(size == 4*float_size)
            {
                float x,y,d,h;
                memcpy(&x, data, float_size);
                memcpy(&y, data+float_size, float_size);
                memcpy(&d, data+2*float_size, float_size);
                memcpy(&h, data+3*float_size, float_size);
                cout<<x<<'\t'<<y<<'\t'<<d<<'\t'<<h<<'\n';
            }
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
}