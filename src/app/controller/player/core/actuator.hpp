#pragma once

#include <iomanip>
#include <iostream>
#include <functional>
#include <thread>
#include <mutex>
#include <list>
#include <unistd.h>
#include "plan/plan.hpp"
#include "sensor/sensor.hpp"
#include "common.hpp"

class actuator
{
public:
    actuator(const std::string &name, const int &max_len = 1, const int us = 1000)
        : name_(name), max_len_(max_len), us_(us)
    {
        is_alive_ = false;
        LOG << std::setw(12) << "actuator:" << std::setw(18) << "[" + name_ + "]" << " started!" << ENDL;
    }

    void add_plan(plan_ptr p)
    {
        if (!is_alive_)
        {
            return;
        }

        plist_mutex_.lock();

        if (plist_.size() >= max_len_)
        {
            plist_.pop_front();
        }

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
        if (td_.joinable())
        {
            td_.join();
        }

        LOG << std::setw(12) << "actuator:" << std::setw(18) << "[" + name_ + "]" << " ended!" << ENDL;
    }

    mutable std::mutex plist_mutex_;
protected:
    void run()
    {
        plan_ptr p;

        while (is_alive_)
        {
            plist_mutex_.lock();
            p.reset();

            if (!plist_.empty())
            {
                p = plist_.front();

                if (!p->perform())
                {
                    LOG << std::setw(12) << "plan:" << std::setw(18) << "[" + p->plan_name() + "]" << " failed!" << ENDL;
                }

                plist_.pop_front();
            }

            plist_mutex_.unlock();
            usleep(us_);
        }
    }
private:
    std::string name_;
    size_t max_len_;
    std::thread td_;
    bool is_alive_;
    int us_;
    std::list<plan_ptr> plist_;
};

typedef std::shared_ptr<actuator> actuator_ptr;
