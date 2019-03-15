#pragma once

#include <mutex>
#include <atomic>
#include "pattern.hpp"
#include "sensor/imu.hpp"
#include "sensor/motor.hpp"
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

    robot_math::transform_matrix head_matrix()
    {
        std::lock_guard<std::mutex> lk(head_mtx_);
        return head_matrix_;
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
        std::lock_guard<std::mutex> lk(hr_mtx_);
        return player_infos_;
    }

    player_info my_info()
    {
        std::lock_guard<std::mutex> lk(hr_mtx_);
        return player_infos_[CONF->id()];
    }

    bool button_status(int id)
    {
        if(id==1) return bt1_status_;
        else if(id==2) return bt2_status_;
    }
    
private:
    imu::imu_data imu_data_;
    RoboCupGameControlData gc_data_;
    std::map< int, player_info > player_infos_;

    robot_math::transform_matrix body_matrix_, head_matrix_;
    std::vector<double> head_degs_;

    std::atomic_bool bt1_status_;
    std::atomic_bool bt2_status_;
    std::atomic_int fall_direction_;
    std::atomic_int support_foot_;

    mutable std::mutex imu_mtx_, dxl_mtx_, head_mtx_, gc_mtx_, hr_mtx_;;
};

#define WM WorldModel::instance()

