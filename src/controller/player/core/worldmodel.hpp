#pragma once

#include <mutex>
#include <atomic>
#include "pattern.hpp"
#include "sensor/imu.hpp"
#include "sensor/button.hpp"
#include "sensor/gamectrl.hpp"
#include "sensor/hear.hpp"
#include "configuration.hpp"
#include "singleton.hpp"
#include "model.hpp"
#include "math/math.hpp"
#include "robot/humanoid.hpp"

class WorldModel: public subscriber, public singleton<WorldModel>
{
public:
    WorldModel();

    void updata(const pub_ptr &pub, const int &type);

    int support_foot()
    {
        return support_foot_;
    }

    void set_support_foot(const robot::support_foot &sf)
    {
        support_foot_ = sf;
    }

    imu::imu_data imu_data()
    {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        return imu_data_;
    }

    int fall_data()
    {
        return fall_direction_;
    }

    RoboCupGameControlData gc_data()
    {
        std::lock_guard<std::mutex> lk(gc_mtx_);
        return gc_data_;
    }

    std::map< int, player_info > player_infos()
    {
        std::lock_guard<std::mutex> lk(info_mtx_);
        return player_infos_;
    }

    player_info my_info()
    {
        std::lock_guard<std::mutex> lk(info_mtx_);
        return player_infos_[CONF->id()];
    }

    void set_ball_pos(const Eigen::Vector2d &gloabal, const Eigen::Vector2d &my, const Eigen::Vector2d &cm)
    {
        std::lock_guard<std::mutex> lk(info_mtx_);
        player_infos_[CONF->id()].ball_x = gloabal.x();
        player_infos_[CONF->id()].ball_y = gloabal.y();
        ball_in_my_space_ = my;
        ball_in_camera_ = cm;
    }

    Eigen::Vector2d ball_in_my_space()
    {
        std::lock_guard<std::mutex> lk(info_mtx_);
        return ball_in_my_space_;
    }

    Eigen::Vector2d ball_in_camera()
    {
        std::lock_guard<std::mutex> lk(info_mtx_);
        return ball_in_camera_;
    }

    void set_my_pos(const Eigen::Vector3d &my)
    {
        std::lock_guard<std::mutex> lk(info_mtx_);
        player_infos_[CONF->id()].x = my.x();
        player_infos_[CONF->id()].y = my.y();
        player_infos_[CONF->id()].dir = my.z();
    }

    bool button_status(int id)
    {
        if(id==1) return bt1_status_;
        else if(id==2) return bt2_status_;
    }

    void navigation(const Eigen::Vector3d &walk_para);
    
private:
    imu::imu_data imu_data_;
    RoboCupGameControlData gc_data_;
    std::map< int, player_info > player_infos_;
    Eigen::Vector2d ball_in_my_space_;
    Eigen::Vector2d ball_in_camera_;

    std::atomic_bool bt1_status_;
    std::atomic_bool bt2_status_;
    std::atomic_int fall_direction_;
    std::atomic_int support_foot_;

    double coef_x_, coef_y_, coef_d_;
    mutable std::mutex imu_mtx_, gc_mtx_, info_mtx_;
};

#define WM WorldModel::instance()

