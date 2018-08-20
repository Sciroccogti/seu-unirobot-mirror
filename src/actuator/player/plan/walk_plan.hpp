#ifndef SEU_UNIROBOT_ACTUATOR_WALK_PLAN_HPP
#define SEU_UNIROBOT_ACTUATOR_WALK_PLAN_HPP

#include "plan.hpp"

class walk_plan: public plan
{
public:
    walk_plan(const float &x, const float &y, const float &dir, const float &h=0.04)
    {

    }

    int perform(sensor_ptr s)
    {
        return 0;
    }
};

#endif
