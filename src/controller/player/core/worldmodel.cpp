#include "worldmodel.hpp"

using namespace Eigen;
using namespace robot_math;
using namespace robot;


WorldModel::WorldModel()
{
    fall_direction_ = FALL_NONE;
    support_foot_ = robot::DOUBLE_SUPPORT;
    player_infos_[CONF->id()].id = CONF->id();
    player_infos_[CONF->id()].x = 0.0;
    player_infos_[CONF->id()].y = -0.75;
    player_infos_[CONF->id()].dir = 0.0;
    player_infos_[CONF->id()].ball_x = 0.0;
    player_infos_[CONF->id()].ball_y = 0.0;
}

void WorldModel::updata(const pub_ptr &pub, const int &type)
{
    if (type == sensor::SENSOR_IMU)
    {
        imu_mtx_.lock();
        std::shared_ptr<imu> sptr = std::dynamic_pointer_cast<imu>(pub);
        imu_data_ = sptr->data();
        fall_direction_ = sptr->fall_direction();
        imu_mtx_.unlock();
        return;
    }

    if(type == sensor::SENSOR_BUTTON)
    {
        std::shared_ptr<button> sptr = std::dynamic_pointer_cast<button>(pub);
        bt1_status_ = sptr->button_1();
        bt2_status_ = sptr->button_2();
        return;
    }

    if(type == sensor::SENSOR_GC)
    {
        gc_mtx_.lock();
        std::shared_ptr<gamectrl> sptr = std::dynamic_pointer_cast<gamectrl>(pub);
        gc_data_ = sptr->data();
        gc_mtx_.unlock();
        return;
    }

    if(type == sensor::SENSOR_HEAR)
    {
        hr_mtx_.lock();
        std::shared_ptr<hear> sptr = std::dynamic_pointer_cast<hear>(pub);
        player_info info = sptr->info();
        player_infos_[info.id] = info;
        hr_mtx_.unlock();
        return;
    }
}