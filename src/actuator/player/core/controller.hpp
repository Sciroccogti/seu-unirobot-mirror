#ifndef SEU_UNIROBOT_CONTROLLER_HPP
#define SEU_UNIROBOT_CONTROLLER_HPP

#include <iostream>
#include <list>
#include <thread>
#include <mutex>
#include "logger.hpp"
#include "../plan/plan.hpp"

class controller
{
public:
    controller(const int &max_len=1): max_len_(max_len)
    {
        
    }
    
    void add_plan(std::list<plan_ptr> plist)
    {
        std::lock_guard<std::mutex> lk(p_mutex_);
        for(auto p:plist)
        {
            if(plan_list_.size()>= max_len_)
                plan_list_.pop_front();
            plan_list_.push_back(p);
        }
    }
    
    void start()
    {
        is_alive_ = true;
        td_ = std::thread(std::bind(&controller::run, this));
    }
    
    void stop()
    {
        is_alive_ = false;
        if(td_.joinable()) td_.join();
    }
    
    void run()
    {
        plan_ptr p;
        while(is_alive_)
        {
            p.reset();
            std::lock_guard<std::mutex> lk(p_mutex_);
            if(!plan_list_.empty())
            {
                p = plan_list_.front();
                if(!p->perform()) LOG(LOG_WARN, "plan perform error: "+p->name());
                plan_list_.pop_front();
            }
        }
        LOG(LOG_INFO, "controller end!");
    }
private:
    std::string name_;
    int max_len_;
    bool is_alive_;
    std::thread td_;
    std::list<plan_ptr> plan_list_;
    mutable std::mutex p_mutex_;
};

#endif