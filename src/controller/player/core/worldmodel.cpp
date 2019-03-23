#include "worldmodel.hpp"

using namespace Eigen;
using namespace robot_math;
using namespace robot;
using namespace std;

WorldModel::WorldModel()
{
    fall_direction_ = FALL_NONE;
    support_foot_ = robot::DOUBLE_SUPPORT;
    player_infos_[CONF->id()].id = CONF->id();
    vector<float> init_pos = CONF->get_config_vector<float>(CONF->player()+".strategy.init");
    player_infos_[CONF->id()].x = init_pos[0];
    player_infos_[CONF->id()].y = init_pos[1];
    player_infos_[CONF->id()].dir = init_pos[2];
    player_infos_[CONF->id()].ball_x = 0.0;
    player_infos_[CONF->id()].ball_y = 0.0;
    coef_x_ = CONF->get_config_value<double>(CONF->player()+".nav.cx");
    coef_y_ = CONF->get_config_value<double>(CONF->player()+".nav.cy");
    coef_d_ = CONF->get_config_value<double>(CONF->player()+".nav.cd");
    ball_in_my_space_[0] = 0.0;
    ball_in_my_space_[1] = -init_pos[1];
    ball_in_camera_[0] = 0.0;
    ball_in_camera_[0] = 0.0;
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
        info_mtx_.lock();
        std::shared_ptr<hear> sptr = std::dynamic_pointer_cast<hear>(pub);
        player_info info = sptr->info();
        player_infos_[info.id] = info;
        info_mtx_.unlock();
        return;
    }
}

void WorldModel::navigation(const Eigen::Vector3d &walk_para)
{
    player_info info = my_info();
    Vector2d currpos(info.x, info.y);
    double dir = info.dir+rad2deg(walk_para[2])*coef_d_;
    dir = dir>180.0?-(360.0-dir):dir<-180.0?360.0-dir:dir;
    Vector2d temp=currpos+rotation_mat_2d(-dir)*Vector2d(-walk_para[1]*coef_y_, walk_para[0]*coef_x_);
    set_my_pos(Vector3d(temp[0], temp[1], dir));
}