#ifndef SEU_UNIROBOT_ACTUATOR_JOINTS_PLAN_HPP
#define SEU_UNIROBOT_ACTUATOR_JOINTS_PLAN_HPP

#include "plan.hpp"
#include "robot/humanoid.hpp"
#include "sensor/motor.hpp"
#include "class_exception.hpp"
#include "math/math.hpp"
#include "core/adapter.hpp"

class joints_plan: public plan
{
public:
    joints_plan(const std::map<int, float> &jdegs, const int &act_time, const std::string &actu)
            :plan("action_plan", actu)
    {
        jdegs_ = jdegs;
        act_time_ = act_time;
    }

    bool perform()
    {
        std::map<int, float> latest_deg, diff, jdmap;
        if(actuator_name_ == "body")
            latest_deg = MADT->get_body_degs();
        else if(actuator_name_ == "head")
            latest_deg = MADT->get_head_degs();
        for(auto &jd:latest_deg)
            diff[jd.first] = jdegs_[jd.first] - latest_deg[jd.first];

        for(int i=1;i<=act_time_;i++)
        {
            jdmap.clear();
            for(auto &jd:diff)
                jdmap[jd.first] = latest_deg[jd.first]+i*jd.second/(float)act_time_;
            if(actuator_name_ == "body")
            {
                while(!MADT->body_empty());
                if(!MADT->add_body_degs(jdmap)) return false;
            }
            else if(actuator_name_ == "head")
            {
                while(!MADT->head_empty());
                if(!MADT->add_head_degs(jdmap)) return false;
            }
        }
        return true;
    }
private:
    std::map<int, float> jdegs_;
    int act_time_;
};

#endif