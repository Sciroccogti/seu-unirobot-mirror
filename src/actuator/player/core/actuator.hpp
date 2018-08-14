#ifndef SEU_UNIROBOT_ACTUATOR_HPP
#define SEU_UNIROBOT_ACTUATOR_HPP

#include <thread>
#include <mutex>
#include <queue>
#include "plan/plan.hpp"

class actuator
{
public:
    actuator(const std::string &name, const int &max_len=1)
        : name_(name), max_len_(max_len)
    {
        
    }
    
private:
    std::string name_;
    int max_len_;
    bool is_alive_;
    std::thread td_;
    std::queue<plan_ptr> plan_que_;
};

#endif