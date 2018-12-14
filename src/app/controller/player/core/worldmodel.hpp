#pragma once

#include <mutex>
#include <atomic>
#include "pattern.hpp"
#include "sensor/imu.hpp"
#include "sensor/motor.hpp"
#include "sensor/button.hpp"
#include "configuration.hpp"
#include "singleton.hpp"
#include "model.hpp"

class WorldModel: public subscriber, public singleton<WorldModel>
{
public:
    WorldModel()
    {
        fall_direction_ = FALL_NONE;
        support_foot_ = robot::DOUBLE_SUPPORT;
        lost_ = false;
        bodyx_ = -1.0;
        bodyy_ = 0.0;
        bodydir_ = 15.0;
        ballx_ = 1.0;
        bally_ = -1.0;
    }

    void updata(const pub_ptr &pub, const int &type)
    {
        if (type == sensor::SENSOR_IMU)
        {
            imu_mtx_.lock();
            std::shared_ptr<imu> sptr = std::dynamic_pointer_cast<imu>(pub);
            imu_data_ = sptr->data();
            lost_ = sptr->lost();
            fall_direction_ = sptr->fall_direction();
            imu_mtx_.unlock();
            return;
        }

        if (type == sensor::SENSOR_MOTOR)
        {
            dxl_mtx_.lock();
            std::shared_ptr<motor> sptr = std::dynamic_pointer_cast<motor>(pub);
            dxl_mtx_.unlock();
            return;
        }

        if(type == sensor::SENSOR_BUTTON)
        {
            std::shared_ptr<button> sptr = std::dynamic_pointer_cast<button>(pub);
            bt1_status_ = sptr->button_1();
            bt2_status_ = sptr->button_2();
        }
    }

    inline int support_foot() const
    {
        return support_foot_;
    }

    inline void set_support_foot(const robot::support_foot &sf)
    {
        support_foot_ = sf;
    }

    inline imu::imu_data imu_data() const
    {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        return imu_data_;
    }

    inline int fall_data() const
    {
        return fall_direction_;
    }

    inline bool lost() const
    {
        return lost_;
    }

public:
    float ballx_, bally_;
    float bodyx_, bodyy_, bodydir_;
    
private:
    imu::imu_data imu_data_;

    std::atomic_bool lost_;
    std::atomic_bool bt1_status_;
    std::atomic_bool bt2_status_;
    std::atomic_int fall_direction_;
    std::atomic_int support_foot_;

    mutable std::mutex imu_mtx_, dxl_mtx_;
};

#define WM WorldModel::instance()

