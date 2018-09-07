#include <list>
#include "player.hpp"
#include "plan/action_plan.hpp"
#include "plan/lookat_plan.hpp"
#include "plan/walk_plan.hpp"
#include "tcp.hpp"

using namespace robot;
using namespace std;

list<plan_ptr> player::play_with_remote()
{
    list<plan_ptr> plist;
    remote_data rdata = wm_->rmt_data();
    if(rdata.type == WALK_DATA)
    {
        float x,y,d,h;
        memcpy(&x, rdata.data.c_str(), float_size);
        memcpy(&y, rdata.data.c_str()+float_size, float_size);
        memcpy(&d, rdata.data.c_str()+2*float_size, float_size);
        memcpy(&h, rdata.data.c_str()+3*float_size, float_size);
        plist.push_back(make_shared<walk_plan>(x,y,d,h));
    }
    else if(rdata.type == ACT_DATA)
    {
        int blksz = int_size + ROBOT->get_joint_map().size()*(int_size+float_size);
        int pos_count = rdata.size/blksz;
        int j_count = ROBOT->get_joint_map().size();
        map<int, float> jdmap;
        int act_t;
        int id;
        float deg;
        vector< map<int, float> > poses;
        vector<int> pos_times;

        for(int i=0;i<pos_count;i++)
        {
            memcpy(&act_t, rdata.data.c_str()+i*blksz, int_size);
            for(int k=0;k<j_count;k++)
            {
                memcpy(&id, rdata.data.c_str()+i*blksz+int_size+k*(int_size+float_size),  int_size);
                memcpy(&deg, rdata.data.c_str()+i*blksz+int_size+k*(int_size+float_size)+int_size,  float_size);
                jdmap[id] = deg;
            }
            pos_times.push_back(act_t);
            poses.push_back(jdmap);
        }
        plist.push_back(make_shared<action_plan>(poses, pos_times));
    }
    else if(rdata.type == LOOKAT_DATA)
    {
        float yaw, pitch;
        memcpy(&yaw, rdata.data.c_str(), float_size);
        memcpy(&pitch, rdata.data.c_str()+float_size, float_size);
        plist.push_back(make_shared<lookat_plan>(yaw, pitch));
    }
    else if(rdata.type == JOINT_OFFSET)
    {
        robot_joint_deg jdeg;
        for(int i=0;i<ROBOT->get_joint_map().size();i++)
        {
            memcpy(&jdeg, rdata.data.c_str()+i*sizeof(robot_joint_deg), sizeof(robot_joint_deg));
            ROBOT->get_joint(jdeg.id)->offset_ = jdeg.deg;
        }
    }
    wm_->reset_rmt_data();
    return plist;
}

