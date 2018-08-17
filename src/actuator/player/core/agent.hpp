#ifndef SEU_UNIROBOT_ACTUATOR_AGENT_HPP
#define SEU_UNIROBOT_ACTUATOR_AGENT_HPP

#include <memory>
#include "timer.hpp"
#include "controller.hpp"
#include "robot_subscriber.hpp"
#include "robot/humanoid.hpp"

class agent: public timer
{
public:
    agent(const int& ms): timer(ms)
    {
        is_alive_ = false;
        suber_ = std::make_shared<robot_subscriber>();
        controller_ = std::make_shared<controller>();
        period_count_ = 0;
    }
    
    bool init()
    {
        if(!suber_->regist()) return false;
        is_alive_ = true;
        controller_->start();
        return true;
    }
    
    void kill()
    {
        is_alive_ = false;
        delete_timer();
        sleep(1);
        suber_->unregist();
        controller_->stop();
    }
    
    void run()
    {
        if(is_alive_)
        {
            period_count_++;
            controller_->add_plan(think());
        }
    }
    
    virtual std::list<plan_ptr> think()=0;
    bool is_alive() const
    {
        return is_alive_;
    }

protected:
    bool is_alive_;
    unsigned long period_count_;
    std::shared_ptr<robot_subscriber> suber_;
    std::shared_ptr<controller> controller_;
};

#endif