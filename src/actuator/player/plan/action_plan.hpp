#ifndef SEU_UNIROBOT_ACTION_PLAN_HPP
#define SEU_UNIROBOT_ACTION_PLAN_HPP

#include "plan.hpp"
#include "robot/humanoid.hpp"
#include "../sensor/motor.hpp"
#include "class_exception.hpp"
class action_plan: public plan
{
public:
    action_plan(const std::string& act_name, sensor_ptr s)
        :plan("action_plan"), act_name_(act_name)
    {
        if(s == nullptr) throw class_exception<action_plan>("emptr sensor ptr");
        motor_ = std::dynamic_pointer_cast<motor>(s);
    }
    
    bool perform()
    {
        auto aiter = robot::ROBOT.get_act_map().find(act_name_);
        if(aiter == robot::ROBOT.get_act_map().end())
        {
            std::cout<<"\033[31mcannot find action: "<<act_name_<<"\033[0m\n";
            return false;
        }
        int act_t;
        std::string pos_name;
        std::map<std::string, float> jdegs;
        std::map<int, float> one_pos_deg, latest_deg, diff;
        std::map<int, float> jdmap;
        
        for(auto p:aiter->second.poses)
        {
            act_t = p.act_time;
            pos_name = p.pos_name;
            jdegs.clear();
            jdegs = robot::ROBOT.get_pos_map()[pos_name].joints_deg;
            one_pos_deg.clear();
            for(auto j:jdegs)
                one_pos_deg[robot::ROBOT.get_joint(j.first)->jid_] = j.second;
            latest_deg = motor_->get_latest_degs();
            diff.clear();
            for(auto jd:one_pos_deg)
                diff[jd.first] = latest_deg[jd.first]-jd.second;
            for(int i=1;i<=act_t;i++)
            {
                jdmap.clear();
                for(auto d:diff)
                    jdmap[d.first] = latest_deg[d.first]+i*d.second/(float)act_t;
                motor_->add_joint_degs(jdmap);
            }
        }
        return true;
    }
private:
    std::string act_name_;
    std::shared_ptr<motor> motor_;
};

#endif