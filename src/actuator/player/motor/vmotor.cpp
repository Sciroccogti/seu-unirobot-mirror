#include "vmotor.hpp"

using namespace std;
using namespace robot;

vmotor::vmotor(sensor_ptr dbg)
{
    dbg_ = std::dynamic_pointer_cast<tcp_server>(dbg);
}

void vmotor::act()
{
    if(dbg_!=nullptr)
    {
        tcp_command cmd;
        cmd.type = JOINT_DATA;
        robot_joint_deg jd;
        string j_data;
        j_data.clear();
        for(auto jm:ROBOT->get_joint_map())
        {
            jd.id = jm.second->jid_;
            jd.deg = jm.second->get_deg();
            j_data.append((char*)(&jd), sizeof(robot_joint_deg));
        }
        cmd.size = ROBOT->get_joint_map().size()*sizeof(robot_joint_deg);
        cmd.data.assign(j_data.c_str(), cmd.size);
        dbg_->write(cmd);
    }
}

vmotor::~vmotor()
{

}
