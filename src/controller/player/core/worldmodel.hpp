#pragma once

#include <mutex>
#include <atomic>
#include "observer.hpp"
#include "sensor/imu.hpp"
#include "sensor/button.hpp"
#include "sensor/gamectrl.hpp"
#include "sensor/hear.hpp"
#include "configuration.hpp"
#include "singleton.hpp"
#include "model.hpp"
#include "math/math.hpp"
#include "robot/humanoid.hpp"
#include "localization/SelfLocalization.h"

class WorldModel: public subscriber, public singleton<WorldModel>
{
public:
    WorldModel();
    
    void updata(const pub_ptr &pub, const int &type);
    void updata_glb();

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

    void set_ball_pos(const Eigen::Vector2d &global, const Eigen::Vector2d &my, const Eigen::Vector2i &pix,
        float alpha, float beta, bool can=true);

    void set_my_pos(const Eigen::Vector2d &my)
    {
        info_mtx_.lock();
        player_infos_[CONF->id()].x = my.x();
        player_infos_[CONF->id()].y = my.y();
        info_mtx_.unlock();
        self_mtx_.lock();
        self_block_.global = my;
        self_mtx_.unlock();
    }

    void set_my_kick(bool kick)
    {
        info_mtx_.lock();
        player_infos_[CONF->id()].my_kick = kick;
        info_mtx_.unlock();
    }

    bool button_status(int id)
    {
        if(id==1) return bt1_status_;
        else if(id==2) return bt2_status_;
    }

    ball_block ball()
    {
        std::lock_guard<std::mutex> lk(ball_mtx_);
        return ball_block_;
    }

    self_block self()
    {
        std::lock_guard<std::mutex> lk(self_mtx_);
        return self_block_;
    }

    player_info my_info()
    {
        std::lock_guard<std::mutex> lk(info_mtx_);
        return player_infos_[CONF->id()];
    }

    void reset_my_pos()
    {
        std::lock_guard<std::mutex> lk(info_mtx_);
        player_infos_[CONF->id()].x = init_pos_[0];
        player_infos_[CONF->id()].y = init_pos_[1];
        player_infos_[CONF->id()].ball_x = 0.0;
        player_infos_[CONF->id()].ball_y = 0.0;
        self_block_.global = Eigen::Vector2d(init_pos_[0], init_pos_[1]);
    }

    void navigation(const Eigen::Vector3d &walk_para);
    void reset_hear_info();

public:
    std::atomic_bool self_localization_;
    std::atomic_bool kickoff_;
    Eigen::Vector2d opp_post_left, opp_post_right;
    
private:
    imu::imu_data imu_data_;
    RoboCupGameControlData gc_data_;
    std::map< int, player_info > player_infos_;
    ball_block ball_block_;
    self_block self_block_;

    std::atomic_bool bt1_status_;
    std::atomic_bool bt2_status_;
    std::atomic_int fall_direction_;
    std::atomic_int support_foot_;

    double coef_x_, coef_y_, coef_d_;
    std::vector<float> init_pos_;
    mutable std::mutex imu_mtx_, gc_mtx_, info_mtx_, ball_mtx_, self_mtx_;
};

#define WM WorldModel::instance()

