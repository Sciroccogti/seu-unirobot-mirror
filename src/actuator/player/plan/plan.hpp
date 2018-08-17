#ifndef SEU_UNIROBOT_ACTUATOR_PLAN_HPP
#define SEU_UNIROBOT_ACTUATOR_PLAN_HPP

#include <memory>
#include "sensor/sensor.hpp"

class plan
{
public:
    plan(const std::string &plan_name, const std::string &actuator_name)
    : plan_name_(plan_name), actuator_name_(actuator_name)
    {
    }
    
    virtual bool perform()
    {
        return true;
    }
    
    std::string plan_name() const
    {
        return plan_name_;
    }

    std::string actuator_name() const
    {
        return actuator_name_;
    }

protected:
    std::string plan_name_;
    std::string actuator_name_;
};

typedef std::shared_ptr<plan> plan_ptr;

#endif