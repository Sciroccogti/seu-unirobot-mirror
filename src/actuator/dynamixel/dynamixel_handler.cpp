#include <string>
#include "dynamixel_handler.hpp"
#include "configuration/configuration.hpp"

#define ADDR_ID     7
#define ADDR_TORQ   64
#define ADDR_LED    65
#define ADDR_GPOS   116
#define ADDR_PPOS   132
#define ADDR_VOLT   144

#define SIZE_ID     1
#define SIZE_TORQ   1
#define SIZE_LED    1
#define SIZE_GPOS   4
#define SIZE_PPOS   4
#define SIZE_VOLT   2

#define MAX_VOLTAGE 16.8
#define ZERO_POS    2048

using namespace std;
using namespace robot;
using namespace config;

dynamixel_handler::dynamixel_handler(): timer(CONF.get_config_value<int>("dynamixel.period")),
    dxl_(CONF.get_config_value<string>("dynamixel.dev_name"), CONF.get_config_value<int>("dynamixel.baudrate"),
        CONF.get_config_value<int>("dynamixel.recv_timeout"))
{
    count_ = 0;
}

void dynamixel_handler::run()
{
    sync_write_pos(joints_datas_.front());
    joints_datas_.pop();
    count_++;
    if(count_%50 == 0)
    {
        led_state_ = 1 - led_state_;
        sync_write_led(led_state_);
    }
    if(count_%1500)
    {
        count_ = 0;
        float v = read_voltage();
    }
}

void dynamixel_handler::add_joint_data(robot::joint_map j_map)
{
    joints_datas_.push(j_map);
}

uint32_t dynamixel_handler::float2uint32(const float &deg)
{
    return static_cast<uint32_t>(deg/(360.0/4096.0)+2048);
}

void dynamixel_handler::sync_write_torq(uint8_t state)
{
    joint_map j_map = ROBOT.get_joint_map();
    uint8_t size_each = SIZE_ID+SIZE_TORQ;
    uint16_t size = j_map.size()*size_each;
    uint8_t data[TXPACKET_MAX_LEN];
    auto iter = j_map.begin();
    int i=0;
    while(iter!=j_map.end())
    {
        data[0+i*size_each] = static_cast<uint8_t>(iter->second->id_);
        data[1+i*size_each] = state;
        iter++;
        i++;
    }
    dxl_.sync_write(ADDR_TORQ, SIZE_TORQ, data, size);
}

void dynamixel_handler::sync_write_pos(robot::joint_map j_map)
{
    uint8_t size_each = SIZE_ID+SIZE_GPOS;
    uint16_t size = j_map.size()*size_each;
    uint8_t data[TXPACKET_MAX_LEN];
    uint32_t pos;
    auto iter = j_map.begin();
    int i=0;
    while(iter!=j_map.end())
    {
        pos = float2uint32(iter->second->get_deg());
        data[0+i*size_each] = static_cast<uint8_t>(iter->second->id_);
        data[1+i*size_each] = (uint8_t)(pos);
        data[2+i*size_each] = (uint8_t)(pos>>8);
        data[3+i*size_each] = (uint8_t)(pos>>16);
        data[4+i*size_each] = (uint8_t)(pos>>24);
        iter++;
        i++;
    }
    dxl_.sync_write(ADDR_GPOS, SIZE_GPOS, data, size);
}

void dynamixel_handler::sync_write_led(uint8_t state)
{
    joint_map j_map = ROBOT.get_joint_map();
    uint8_t size_each = SIZE_ID+SIZE_LED;
    uint16_t size = j_map.size() *size_each;
    uint8_t data[TXPACKET_MAX_LEN];
    int i=0;
    auto iter = j_map.begin();
    while(iter!=j_map.end())
    {
        data[0+i*size_each] = static_cast<uint8_t>(iter->second->id_);
        data[1+i*size_each] = state;
        iter++;
        i++;
    }
    dxl_.sync_write(ADDR_LED, SIZE_LED, data, size);
}

float dynamixel_handler::read_voltage()
{
    uint8_t data[SIZE_VOLT];
    if(dxl_.read(static_cast<uint8_t>(ROBOT.get_joint_map()["jrhip1"]->id_), ADDR_VOLT, SIZE_VOLT, data))
        return ((data[1]<<8)|data[0])/10.0;
    else return MAX_VOLTAGE;
}