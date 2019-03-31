#pragma once

#include <list>
#include "timer.hpp"
#include "core/worldmodel.hpp"
#include "robot/humanoid.hpp"
#include "options/options.hpp"
#include "sensor/imu.hpp"
#include "sensor/motor.hpp"
#include "sensor/camera.hpp"
#include "sensor/button.hpp"
#include "vision/vision.hpp"
#include "common.hpp"
#include "task/task.hpp"

class player: public timer, public std::enable_shared_from_this<player>
{
public:
    enum PlayerRole
    {
        KEEPER = 0,
        GUARD = 1,
        ATTACKER = 2
    };

    enum RobotState
    {
        STATE_NOTMAL=1,
        STATE_GETUP=2,
        STATE_KICK=3,
        STATE_SEARCH=4
    };
    
    static const std::map<std::string, PlayerRole> RoleName;
public:
    player();
    bool init();
    void stop();
    bool is_alive() const
    {
        return is_alive_;
    }

private:
    void run();
    void play_with_remote();
    std::list<task_ptr> play_with_gc();
    std::list<task_ptr> play_without_gc();
    std::list<task_ptr> think();
    bool regist();
    void unregist();
    sensor_ptr get_sensor(const std::string &name);
private:
    unsigned long period_count_;
    std::map<std::string, sensor_ptr> sensors_;
    unsigned int btn_count_;
    PlayerRole role_;
    RobotState state_;
    int w_, h_;
};
