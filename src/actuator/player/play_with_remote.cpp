#include <list>
#include "player.hpp"
#include "plan/action_plan.hpp"

using namespace comm;
using namespace robot;
using namespace std;

list<plan_ptr> player::play_with_remote()
{
    list<plan_ptr> plist;
    int size_float = sizeof(float), size_int = sizeof(int);
    tcp_packet::remote_data rdata = suber_->rmt_data();
    //LOG(LOG_INFO, "TYPE: "<<(int)rdata.type);
    if(rdata.type == tcp_packet::WALK_DATA)
    {
        float x,y,d,h;
        memcpy(&x, rdata.data.c_str(), size_float);
        memcpy(&y, rdata.data.c_str()+size_float, size_float);
        memcpy(&d, rdata.data.c_str()+2*size_float, size_float);
        memcpy(&h, rdata.data.c_str()+3*size_float, size_float);
    }
    else if(rdata.type == tcp_packet::ACT_DATA)
    {
        int blksz = size_int + ROBOT.get_joint_map().size()*(size_int+size_float);
        int pos_count = rdata.size/blksz;
        int j_count = ROBOT.get_joint_map().size();
        map<int, float> jdmap;
        int act_t;
        int id;
        float deg;
        vector< map<int, float> > poses;
        vector<int> pos_times;

        for(int i=0;i<pos_count;i++)
        {
            memcpy(&act_t, rdata.data.c_str()+i*blksz, size_int);
            for(int k=0;k<j_count;k++)
            {
                memcpy(&id, rdata.data.c_str()+i*blksz+size_int+k*(size_int+size_float),  size_int);
                memcpy(&deg, rdata.data.c_str()+i*blksz+size_int+k*(size_int+size_float)+size_int,  size_float);
                jdmap[id] = deg;
            }
            pos_times.push_back(act_t);
            poses.push_back(jdmap);
        }
        plist.push_back(make_shared<action_plan>(poses, pos_times));
    }
    suber_->reset_rmt_data();
    return plist;
}

