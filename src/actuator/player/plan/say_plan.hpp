#ifndef SEU_UNIROBOT_ACTUATOR_SAY_PLAN_HPP
#define SEU_UNIROBOT_ACTUATOR_SAY_PLAN_HPP

#include <boost/asio.hpp>
#include "plan.hpp"
#include "robot/humanoid.hpp"
#include "class_exception.hpp"
#include "configuration.hpp"
#include "model.hpp"
#include "sensor/hear.hpp"

class say_plan: public plan
{
public:
    say_plan(const player_info &info): plan("say_plan","udp")
    {
        info_ = info;
    }
    
    int perform(sensor_ptr s=nullptr)
    {
        std::shared_ptr<hear> h = std::dynamic_pointer_cast<hear>(s);
        h->send(info_);
        return true;
    }
private:
    player_info info_;
};

#endif