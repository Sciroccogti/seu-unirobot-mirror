#ifndef SEU_UNIROBOT_ACTUATOR_LOOKAT_PLAN_HPP
#define SEU_UNIROBOT_ACTUATOR_LOOKAT_PLAN_HPP

#include "plan.hpp"
#include "robot/humanoid.hpp"
#include "joints_plan.hpp"
#include "class_exception.hpp"

class lookat_plan: public plan
{
public:
    lookat_plan(const float &yaw, const float &pitch, const int &act_time=1)
        :plan("lookat_plan","head"), yaw_(yaw), pitch_(pitch), act_time_(act_time)
    {
    }
    
    int perform(sensor_ptr s)
    {
        std::map<int, float> jdmap;

        int pitch_id, yaw_id;
        jdmap[robot::ROBOT->get_joint("jhead1")->jid_] = yaw_;
        jdmap[robot::ROBOT->get_joint("jhead2")->jid_] = pitch_;
        joints_plan jp(jdmap, act_time_, actuator_name_);
        return jp.perform(s);
    }
private:
    float pitch_, yaw_, act_time_;
};

#endif