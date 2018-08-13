#ifndef SEU_UNIROBOT_DYNAMIXEL_HPP
#define SEU_UNIROBOT_DYNAMIXEL_HPP

#include <memory>
#include "dynamixel/dynamixel_sdk.h"
#include "timer.hpp"
#include "sensor.hpp"

class dxl: public sensor, public timer
{
public:
    dxl(const sub_ptr &s);
    ~dxl();
    
    bool start();
    void run();
    bool open();
    void close();
    
private:
    std::shared_ptr<dynamixel::PortHandler> portHandler_;
    std::shared_ptr<dynamixel::PacketHandler> packetHandler_;
    int period_;
    unsigned long p_count_;
};

#endif