#pragma once

#include <list>
#include "timer.hpp"
#include "core/worldmodel.hpp"
#include "core/actuator.hpp"
#include "fsm/fsm.hpp"
#include "robot/humanoid.hpp"
#include "options/options.hpp"
#include "sensor/imu.hpp"
#include "sensor/motor.hpp"
#include "sensor/gamectrl.hpp"
#include "sensor/server.hpp"
#include "sensor/hear.hpp"
#include "walk/WalkEngine.hpp"
#include "core/vision.hpp"

class player: public timer, public std::enable_shared_from_this<player>
{
public:
    player();
    bool init();
    void stop();
    bool is_alive() const
    {
        return is_alive_;
    }
protected:
    void run();
private:
    std::list<plan_ptr> think();
    std::list<plan_ptr> play_with_remote();
    std::list<plan_ptr> play_with_gamectrl();
    std::list<plan_ptr> play_without_gamectrl();
    void add_plans(std::list<plan_ptr> plist);

    bool regist();
    void unregist();
    sensor_ptr get_sensor(const std::string &name);
private:
    unsigned long period_count_;
    std::map<std::string, actuator_ptr> actuators_;
    std::map<std::string, sensor_ptr> sensors_;
    std::shared_ptr<FSM::fsm> fsm_;
};