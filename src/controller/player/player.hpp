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
#include "fsm/fsm.hpp"

class player: public timer
{
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
    std::string role_;
    unsigned int self_location_count_;
    bool played_;

    FSM_Ptr fsm_;
};
