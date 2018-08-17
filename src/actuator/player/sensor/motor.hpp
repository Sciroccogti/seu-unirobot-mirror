#ifndef SEU_UNIROBOT_ACTUATOR_MOTOR_HPP
#define SEU_UNIROBOT_ACTUATOR_MOTOR_HPP

#include <memory>
#include <list>
#include <mutex>
#include <map>
#include "timer.hpp"
#include "sensor.hpp"
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
        std::lock_guard<std::mutex> lk(bd_mutex_);
        if(!body_degs_list.empty()) return body_degs_list.back();
        else
        {
            std::map<int, float> res;
            for(auto j:robot::ROBOT.get_joint_map())
            {
                if(j.second->name_.find("head") == std::string::npos)
                    res[j.second->jid_] = j.second->get_deg();
            }
            return res;
        }
    }
    
    std::map<int, float> get_head_degs()
    {
        std::lock_guard<std::mutex> lk(hd_mutex_);
        if(!head_degs_list.empty()) return head_degs_list.back();
        else
        {
            std::map<int, float> res;
            robot::joint_ptr j = robot::ROBOT.get_joint("jhead1");
            res[j->jid_] = j->get_deg();
            j = robot::ROBOT.get_joint("jhead2");
            res[j->jid_] = j->get_deg();
            return res;
        }
    }

    bool body_empty() const
    {
        std::lock_guard<std::mutex> lk(bd_mutex_);
        return body_degs_list.empty();
    }

    bool head_empty() const
    {
        std::lock_guard<std::mutex> lk(hd_mutex_);
        return head_degs_list.empty();
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