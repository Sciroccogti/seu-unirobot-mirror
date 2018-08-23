#ifndef SEU_UNIROBOT_ACTUATOR_CONTROLLER_HPP
#define SEU_UNIROBOT_ACTUATOR_CONTROLLER_HPP

#include <list>
#include <map>
#include "actuator.hpp"
#include "plan/plan.hpp"
#include "robot_subscriber.hpp"
#include "options/options.hpp"

class controller
{
public:
    controller(std::shared_ptr<robot_subscriber> suber)
    {
        if(OPTS.use_robot() != options::ROBOT_NONE)
        {
            actuators_["body"] = std::make_shared<actuator>("body", suber->get_sensor("motor"));
            actuators_["head"] = std::make_shared<actuator>("head", suber->get_sensor("motor"));
            actuators_["udp"] = std::make_shared<actuator>("udp");
        }
    }
    
    void add_plan(std::list<plan_ptr> plist)
    {
        for(auto &p:plist)
        {
            if(actuators_.find(p->actuator_name()) != actuators_.end())
            {
                actuator_ptr actu = actuators_[p->actuator_name()];
                actu->add_plan(p);
            }
            else
            {
                std::cout<<"cannot find actuator: "+p->actuator_name()<<"\n";
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
        for(auto &a:actuators_)
            a.second->stop();
    }

private:
    std::map<std::string, actuator_ptr> actuators_;
};

#endif