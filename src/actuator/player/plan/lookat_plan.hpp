#pragma once

#include "plan.hpp"
#include "robot/humanoid.hpp"
#include "joints_plan.hpp"
#include "class_exception.hpp"

class lookat_plan: public plan
{
public:
    lookat_plan(const float &yaw, const float &pitch, const int &act_time = 1)
        : plan("lookat_plan", "head")
    {
        act_time_ = act_time;
        yaw_ = yaw;
        pitch_ = pitch;
    }

    bool perform()
    {
        std::map<int, float> jdmap;
        jdmap[robot::ROBOT->get_joint("jhead1")->jid_] = yaw_;
        jdmap[robot::ROBOT->get_joint("jhead2")->jid_] = pitch_;
        joints_plan jp(jdmap, act_time_, actuator_name_);
        return jp.perform();
    }
private:
    float pitch_;
    float yaw_;
    int act_time_;
};
