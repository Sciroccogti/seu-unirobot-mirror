#ifndef SEU_UNIROBOT_PLAYER_HPP
#define SEU_UNIROBOT_PLAYER_HPP

#include <list>
#include "timer.hpp"
#include "core/worldmodel.hpp"
#include "core/actuator.hpp"
#include "fsm/fsm.hpp"
#include "robot/humanoid.hpp"
#include "options/options.hpp"
#include "sensor/imu.hpp"
#include "motor/rmotor.hpp"
#include "motor/vmotor.hpp"
#include "sensor/gamectrl.hpp"
#include "sensor/tcp_server.hpp"
#include "sensor/hear.hpp"
#include "walk/WalkEngine.hpp"
#include "vision/vision.hpp"

class player: public timer, public std::enable_shared_from_this<player>
{
public:
    player();
    bool init();
    void stop();
    bool is_alive() const { return is_alive_; }
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

    std::shared_ptr<worldmodel> wm_;
    std::shared_ptr<walk::WalkEngine> we_;
    std::shared_ptr<vision> vision_;
    std::shared_ptr<FSM::fsm> fsm_;
};

#endif