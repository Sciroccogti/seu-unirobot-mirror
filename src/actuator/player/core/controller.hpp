#ifndef SEU_UNIROBOT_ACTUATOR_CONTROLLER_HPP
#define SEU_UNIROBOT_ACTUATOR_CONTROLLER_HPP

#include <list>
#include <map>
#include "actuator.hpp"
#include "plan/plan.hpp"
#include "robot_subscriber.hpp"
#include "logger.hpp"
#include "options/options.hpp"

class controller
{
public:
    controller(std::shared_ptr<robot_subscriber> suber)
    {
        if(OPTS.use_robot() != options::ROBOT_NONE)
        {
            actuators_["body"] = std::make_shared<actuator>(suber->get_sensor("motor"), "body");
            actuators_["head"] = std::make_shared<actuator>(suber->get_sensor("motor"), "head");
        }
    }
    
    void add_plan(std::list<plan_ptr> plist)
    {
        for(auto p:plist)
        {
            if(actuators_.find(p->actuator_name()) != actuators_.end())
            {
                actuator_ptr actu = actuators_[p->actuator_name()];
                actu->add_plan(p);
            }
            else
            {
                LOG(LOG_WARN, "cannot find actuator: "+p->actuator_name());
            }
        }
    }

    void start()
    {
        for(auto a:actuators_)
            a.second->start();
    }

    void stop()
    {
        for(auto a:actuators_)
            a.second->stop();
    }

private:
    std::map<std::string, actuator_ptr> actuators_;
};

#endif