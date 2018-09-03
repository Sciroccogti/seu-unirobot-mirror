#ifndef SEU_UNIROBOT_ACTUATOR_MOTOR_HPP
#define SEU_UNIROBOT_ACTUATOR_MOTOR_HPP

#include <memory>
#include <list>
#include <mutex>
#include <map>
#include "timer.hpp"
#include "sensor/sensor.hpp"
#include "robot/humanoid.hpp"

class motor: public sensor, public timer
{
public:
    motor();
    ~motor();

    virtual void act()=0;

    virtual bool open()
    {
        is_open_ = true;
        return true;
    }

    virtual void close()
    {
        is_open_ = false;
    }

    void stop();
    bool start();
    void run();
    
    bool add_body_degs(const std::map<int, float> &jdmap);
    bool add_head_degs(const std::map<int, float> &jdmap);
    
    std::map<int, float> get_body_degs()
    {
        bd_mutex_.lock();
        std::map<int, float> res;
        if(!body_degs_list.empty())
            res = body_degs_list.back();
        else
        {
            for(auto j:robot::ROBOT.get_joint_map())
            {
                if(j.second->name_.find("head") == std::string::npos)
                    res[j.second->jid_] = j.second->get_deg();
            }
        }
        bd_mutex_.unlock();
        return res;
    }
    
    std::map<int, float> get_head_degs()
    {
        hd_mutex_.lock();
        std::map<int, float> res;
        if(!head_degs_list.empty())
            res = head_degs_list.back();
        else
        {
            robot::joint_ptr j = robot::ROBOT.get_joint("jhead1");
            res[j->jid_] = j->get_deg();
            j = robot::ROBOT.get_joint("jhead2");
            res[j->jid_] = j->get_deg();
        }
        hd_mutex_.unlock();
        return res;
    }

    bool body_empty() const
    {
        bd_mutex_.lock();
        bool res = body_degs_list.empty();
        bd_mutex_.unlock();
        return res;
    }

    bool head_empty() const
    {
        hd_mutex_.lock();
        bool res = head_degs_list.empty();
        hd_mutex_.unlock();
        return res;
    }

    double freq() const
    {
        return 1000.0/(double)period_;
    }

    mutable std::mutex bd_mutex_, hd_mutex_;
protected:
    unsigned long p_count_;
    int period_;
private:
    std::list< std::map<int, float> > head_degs_list;
    std::list< std::map<int, float> > body_degs_list;
};

#endif