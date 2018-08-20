#ifndef SEU_UNIROBOT_ACTUATOR_RMOTOR_HPP
#define SEU_UNIROBOT_ACTUATOR_RMOTOR_HPP

#include <memory>
#include "dynamixel/dynamixel_sdk.h"
#include "motor.hpp"

#define MAX_VOLTAGE 16.8
#define ZERO_POS 2048
#define MAX_POS  4096
#define MIN_POS  0

class rmotor: public motor
{
public:
    rmotor(const sub_ptr &s);
    ~rmotor();

    bool open();
    void close();
    void act();
    
    float voltage() const
    {
        return voltage_/10.0;
    }
private:
    void set_torq(uint8_t e);
    void set_led(uint8_t s);
    void set_gpos();
    
    std::shared_ptr<dynamixel::PortHandler> portHandler_;
    std::shared_ptr<dynamixel::PacketHandler> packetHandler_;
    
    std::shared_ptr<dynamixel::GroupSyncWrite> torqWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> gposWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> ledWrite_;

    uint16_t voltage_;
    uint8_t dxl_error_;
    uint8_t led_status_;
};

#endif