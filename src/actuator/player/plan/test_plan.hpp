#ifndef SEU_UNIROBOT_ACTUATOR_TEST_PLAN_HPP
#define SEU_UNIROBOT_ACTUATOR_TEST_PLAN_HPP

#include "plan.hpp"

class test_plan: public plan
{
public:
    test_plan():plan("testplan")
    {
    }
    
    bool perform()
    {
        std::cout<<"test plan\n";
        return true;
    }
};

#endif