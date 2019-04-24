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

    task_ptr play_skill_goto(const Eigen::Vector2d &target, double dir);
    task_ptr play_skill_penalty_kick(bool left, float init_dir);
    std::list<task_ptr> play_skill_kick(const self_block &self, const ball_block &ball);

    std::list<task_ptr> play_skill_search_ball();

    std::list<task_ptr> play_skill_localization();

    bool in_my_attack_range(const Eigen::Vector2d &ball);
    bool regist();
    void unregist();
    sensor_ptr get_sensor(const std::string &name);
private:
    unsigned long period_count_;
    std::map<std::string, sensor_ptr> sensors_;
    unsigned int btn_count_;
    std::string role_;
    std::vector<float> attack_range_;
    double last_search_dir_;
    bool in_search_ball_;
    bool see_last_;
    unsigned int self_location_count_;
    bool played_;
    bool keeper_kicked_;
};
