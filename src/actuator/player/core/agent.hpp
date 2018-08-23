#ifndef SEU_UNIROBOT_ACTUATOR_AGENT_HPP
#define SEU_UNIROBOT_ACTUATOR_AGENT_HPP

#include <memory>
#include "timer.hpp"
#include "controller.hpp"
#include "robot_subscriber.hpp"
#include "robot/humanoid.hpp"
#include "vision/vision.hpp"

class agent: public timer
{
public:
    agent(const int& ms): timer(ms)
    {
        is_alive_ = false;
        period_count_ = 0;
    }

    bool init()
    {
        suber_ = std::make_shared<robot_subscriber>();
        if(!suber_->start()) return false;
        
        controller_ = std::make_shared<controller>(suber_);
        controller_->start(); 
        
        if(OPTS.use_camera())
        {
            vision_ = std::make_shared<vision>(suber_->get_sensor("server"));
            if(!vision_->start()) return false;
        }
        
        is_alive_ = true;
        return true;
    }
    
    void kill()
    {
        if(OPTS.use_camera()) vision_->stop();

        if(controller_!= nullptr) controller_->stop();

        if(is_alive_) delete_timer();
        is_alive_ = false;
        sleep(1);
        suber_->stop();
    }
    
    void run()
    {
        if(is_alive_)
        {
            period_count_++;
            controller_->add_plan(think());
        }
    }
    
    bool is_alive() const
    {
        return is_alive_;
    }
    
    virtual std::list<plan_ptr> think()=0;

protected:
    unsigned long period_count_;
    std::shared_ptr<vision> vision_;
    std::shared_ptr<robot_subscriber> suber_;
    std::shared_ptr<controller> controller_;
};

#endif