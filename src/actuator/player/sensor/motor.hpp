#ifndef SEU_UNIROBOT_MOTOR_HPP
#define SEU_UNIROBOT_MOTOR_HPP

#include <memory>
#include <list>
#include <mutex>
#include "dynamixel/dynamixel_sdk.h"
#include "timer.hpp"
#include "debuger.hpp"
#include "robot/humanoid.hpp"

#define MAX_VOLTAGE 16.8
#define ZERO_POS 2048
#define MAX_POS  4096
#define MIN_POS  0

class motor: public sensor, public timer
{
public:
    motor(const sub_ptr &s, sensor_ptr dbg=nullptr);
    ~motor();
    
    bool open();
    bool start();
    void run();
    void stop();
    
    void add_joint_degs(const std::map<int, float> &jdmap);
    
    float voltage() const
    {
        return voltage_/10.0;
    }
    
    std::map<int, float> get_latest_degs()
    {
        std::lock_guard<std::mutex> lk(jd_mutex_);
        if(!joint_degs_list.empty()) return joint_degs_list.back();
        else
        {
            std::map<int, float> res;
            for(auto j:robot::ROBOT.get_joint_map())
                res[j.second->jid_] = 0.0;
            return res;
        }
    }
    mutable std::mutex jd_mutex_;
private:
    void set_torq(uint8_t e);
    void set_led(uint8_t s);
    void set_gpos();
    
    std::shared_ptr<dynamixel::PortHandler> portHandler_;
    std::shared_ptr<dynamixel::PacketHandler> packetHandler_;
    
    std::shared_ptr<dynamixel::GroupSyncWrite> torqWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> gposWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> ledWrite_;
    
    std::shared_ptr<debuger> dbg_;
    uint16_t voltage_;
    uint8_t dxl_error_;
    int period_;
    uint8_t led_status_;
    unsigned long p_count_;
    std::list< std::map<int, float> > joint_degs_list;
};

#endif