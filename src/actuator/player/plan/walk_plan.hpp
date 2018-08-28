#ifndef SEU_UNIROBOT_ACTUATOR_WALK_PLAN_HPP
#define SEU_UNIROBOT_ACTUATOR_WALK_PLAN_HPP

#include "plan.hpp"
#include "walk/WalkEngine.hpp"
#include "joints_plan.hpp"

class walk_plan: public plan
{
public:
    walk_plan(const float &x, const float &y, const float &dir, const float &h=0.04)
        :plan("walk_plan", "body")
    {
        params_[0] = x;
        params_[1] = y;
        params_[2] = dir;
        params_[3] = h;
    }

    int perform(sensor_ptr s)
    {
        walk::WALK.set_params(params_);
        walk::WALK.set_enable(true);
        return 0;
    }
private:
    Eigen::Vector4f params_;
};

#endif
