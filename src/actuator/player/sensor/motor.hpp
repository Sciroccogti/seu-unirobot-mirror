#ifndef SEU_UNIROBOT_ACTUATOR_MOTOR_HPP
#define SEU_UNIROBOT_ACTUATOR_MOTOR_HPP

#include <memory>
#include <list>
#include <mutex>
#include <map>
#include "timer.hpp"
#include "sensor/server.hpp"
#include "robot/humanoid.hpp"
#include "dynamixel/dynamixel_sdk.h"

class motor: public sensor, public timer
{
public:
    motor(sensor_ptr dbg=nullptr);
    ~motor();
    void stop();
    bool start();
    void run();
    double freq() const { return 1000.0/(double)period_ms_; }
    float voltage() const { return voltage_/10.0; }
private:
    bool open();
    void close();
    void set_torq(uint8_t e);
    void set_led(uint8_t s);
    void set_gpos();
    void virtul_act();
    void real_act();
private:
    unsigned long p_count_;
    uint16_t voltage_;
    bool is_connected_;
    uint8_t ping_id_;
    int lost_count_;
    uint8_t led_status_;

    std::shared_ptr<dynamixel::PortHandler> portHandler_;
    std::shared_ptr<dynamixel::PacketHandler> packetHandler_;
    std::shared_ptr<dynamixel::GroupSyncWrite> torqWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> gposWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> ledWrite_;
    std::shared_ptr<tcp_server> dbg_;
};

#endif