#ifndef SEU_UNIROBOT_AGENT_HPP
#define SEU_UNIROBOT_AGENT_HPP

#include <memory>
#include "timer.hpp"
#include "robot_subscriber.hpp"
#include "robot/humanoid.hpp"

class agent: public timer
{
public:
    agent(const int& ms): timer(ms)
    {
        is_alive_ = false;
        suber_ = std::make_shared<robot_subscriber>();
    }
    
    bool init()
    {
        if(!suber_->regist()) return false;
        is_alive_ = true;
        return true;
    }
    
    void kill()
    {
        delete_timer();
        sleep(1);
        suber_->unregist();
        is_alive_ = false;
    }
    
    void run()
    {
        if(is_alive_)
        {
            if(suber_->rmt_data().type == comm::tcp_packet::WALK_DATA)
            {
                float x,y,d,h;
                memcpy(&x, suber_->rmt_data().data.c_str(), 4);
                memcpy(&y, suber_->rmt_data().data.c_str()+4, 4);
                memcpy(&d, suber_->rmt_data().data.c_str()+2*4, 4);
                memcpy(&h, suber_->rmt_data().data.c_str()+3*4, 4);
                std::cout<<x<<'\t'<<y<<'\t'<<d<<'\t'<<h<<'\n';
            }
            else if(suber_->rmt_data().type == comm::tcp_packet::ACT_DATA)
            {
                int blksz = sizeof(int)+robot::ROBOT.get_joint_map().size()* sizeof(robot::robot_joint_deg);
                for(int k=0;k<suber_->rmt_data().size/blksz;k++)
                {
                    int act_t;
                    memcpy(&act_t, suber_->rmt_data().data.c_str()+k*blksz, 4);
                    robot::robot_joint_deg jd;
                    std::cout<<"act_time: "<<act_t<<'\n';
                    for(int i=0;i<robot::ROBOT.get_joint_map().size();i++)
                    {
                        memcpy(&jd, suber_->rmt_data().data.c_str()+k*blksz+4+i*sizeof(robot::robot_joint_deg), sizeof(robot::robot_joint_deg));
                        std::cout<<jd.id<<'\t'<<jd.deg<<'\n';
                    }
                }
            }
        }
    }
    
    bool is_alive() const
    {
        return is_alive_;
    }

protected:
    bool is_alive_;
    std::shared_ptr<robot_subscriber> suber_;
};

#endif