#ifndef SEU_UNIROBOT_ACTUATOR_JOINTS_PLAN_HPP
#define SEU_UNIROBOT_ACTUATOR_JOINTS_PLAN_HPP

#include "plan.hpp"
#include "robot/humanoid.hpp"
#include "motor/motor.hpp"
#include "class_exception.hpp"
#include "math/math.hpp"

class joints_plan: public plan
{
public:
    joints_plan(const std::map<int, float> &jdegs, const int &act_time, const std::string &actu)
            :plan("action_plan", actu)
    {
        jdegs_ = jdegs;
        act_time_ = act_time;
    }

    int perform(sensor_ptr s)
    {
        std::shared_ptr<motor> motor_ = std::dynamic_pointer_cast<motor>(s);
        std::map<int, float> latest_deg, diff, jdmap;
        if(actuator_name_ == "body")
            latest_deg = motor_->get_body_degs();
        else if(actuator_name_ == "head")
            latest_deg = motor_->get_head_degs();
        for(auto jd:latest_deg)
            diff[jd.first] = jdegs_[jd.first] - latest_deg[jd.first];

        for(int i=1;i<=act_time_;i++)
        {
            jdmap.clear();
            for(auto jd:diff)
                jdmap[jd.first] = latest_deg[jd.first]+i*jd.second/(float)act_time_;
            if(actuator_name_ == "body")
            {
                if(!motor_->add_body_degs(jdmap)) return -1;
                //while (!motor_->body_empty());
            }
            else if(actuator_name_ == "head")
            {
                if(!motor_->add_head_degs(jdmap)) return -1;
                //while (!motor_->head_empty());
            }
        }
        return 0;
    }
private:
    std::map<int, float> jdegs_;
    int act_time_;
};

#endif