#ifndef SEU_UNIROBOT_ACTUATOR_ACTUATOR_HPP
#define SEU_UNIROBOT_ACTUATOR_ACTUATOR_HPP

#include <thread>
#include <mutex>
#include <list>
#include "logger.hpp"
#include "plan/plan.hpp"
#include "sensor/sensor.hpp"

class actuator
{
public:
    actuator(const std::string &name, sensor_ptr s=nullptr, const int &max_len=1)
    :name_(name), max_len_(max_len)
    {
        s_ = s;
        is_alive_ = false;
    }

    void add_plan(plan_ptr p)
    {
        if(!is_alive_) return;
        plist_mutex_.lock();
        if(plist_.size()>=max_len_)
            plist_.pop_front();
        plist_.push_back(p);
        plist_mutex_.unlock();
    }

    void start()
    {
        is_alive_ = true;
        td_ = std::thread(std::bind(&actuator::run, this));
    }

    void stop()
    {
        is_alive_ = false;
    }

    ~actuator()
    {
        if(td_.joinable()) td_.join();
        LOG(LOG_INFO)<<"actuator: [ "+name_+" ] end!\n";
    }

    mutable std::mutex plist_mutex_;
protected:
    void run()
    {
        plan_ptr p;
        while(is_alive_)
        {
            plist_mutex_.lock();
            p.reset();
            if(!plist_.empty())
            {
                p = plist_.front();
                plist_.pop_front();
                if(p->perform(s_) == -1) LOG(LOG_WARN)<<"plan: "+p->plan_name()+" perform failed.\n";
            }
            plist_mutex_.unlock();
        }
    }
private:
    std::string name_;
    int max_len_;
    std::thread td_;
    bool is_alive_;
    std::list<plan_ptr> plist_;
    sensor_ptr s_;
};

typedef std::shared_ptr<actuator> actuator_ptr;

#endif
