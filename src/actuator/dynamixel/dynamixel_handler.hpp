#ifndef SEU_UNIROBOT_DYNAMIXEL_HANDLER_HPP
#define SEU_UNIROBOT_DYNAMIXEL_HANDLER_HPP

#include <queue>
#include <map>
#include "dynamixel.hpp"
#include "timer.hpp"
#include "singleton.hpp"
#include "robot/humanoid.hpp"

class dynamixel_handler: public timer, public singleton<dynamixel_handler>
{
public:
    dynamixel_handler();
    void run();
    void add_joint_data(robot::joint_map j_map);

private:
    uint32_t float2uint32(const float &deg);
    void sync_write_torq(uint8_t state);
    void sync_write_pos(robot::joint_map j_map);
    void sync_write_led(uint8_t state);
    float read_voltage();
    std::queue<robot::joint_map> joints_datas_;
    dynamixel dxl_;
    uint32_t count_;
    uint8_t led_state_;
};

#define DXL dynamixel_handler::get_singleton()

#endif