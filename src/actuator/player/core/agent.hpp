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
        period_count_ = 0;
    }
    
    bool init()
    {
        if(!suber_->regist()) return false;
        controller_ = std::make_shared<controller>(suber_);
        is_alive_ = true;
        controller_->start();
        return true;
    }
    
    void kill()
    {
        if(controller_!= nullptr) controller_->stop();
        if(is_alive_) delete_timer();
        is_alive_ = false;
        sleep(2);
        suber_->unregist();
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