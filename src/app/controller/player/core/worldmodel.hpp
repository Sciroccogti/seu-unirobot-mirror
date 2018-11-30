#pragma once

#include <mutex>
#include "pattern.hpp"
#include "sensor/imu.hpp"
#include "sensor/motor.hpp"
#include "configuration.hpp"
#include "singleton.hpp"

class WorldModel: public subscriber, public singleton<WorldModel>
{
public:
    WorldModel()
    {
        support_foot_ = robot::DOUBLE_SUPPORT;
        low_power_ = false;
        lost_ = false;
        bodyx_ = -1.0;
        bodyy_ = 0.0;
        bodydir_ = 15.0;
        ballx_ = 1.0;
        bally_ = -1.0;
    }

    void updata(const pro_ptr &pub, const int &type)
    {
        if (type == sensor::SENSOR_IMU)
        {
            imu_mtx_.lock();
            std::shared_ptr<imu> sptr = std::dynamic_pointer_cast<imu>(pub);
            imu_data_ = sptr->data();
            sw_data_ = sptr->switch_data();
            lost_ = sptr->lost();
            imu_mtx_.unlock();
            return;
        }

        if (type == sensor::SENSOR_MOTOR)
        {
            dxl_mtx_.lock();
            std::shared_ptr<motor> sptr = std::dynamic_pointer_cast<motor>(pub);
            low_power_ = sptr->low_power();
            dxl_mtx_.unlock();
            return;
        }
    }

    inline robot::support_foot get_support_foot() const
    {
        std::lock_guard<std::mutex> lk(sf_mtx_);
        return support_foot_;
    }

    inline void set_support_foot(const robot::support_foot &sf)
    {
        std::lock_guard<std::mutex> lk(sf_mtx_);
        support_foot_ = sf;
    }

    inline imu::imu_data imu_data() const
    {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        return imu_data_;
    }

    inline sw_data switch_data() const
    {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        return sw_data_;
    }

    inline bool low_power() const
    {
        std::lock_guard<std::mutex> lk(dxl_mtx_);
        return low_power_;
    }

    inline bool lost() const
    {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        return lost_;
    }

    float ballx_, bally_;
    float bodyx_, bodyy_, bodydir_;

private:
    bool low_power_, lost_;
    imu::imu_data imu_data_;
    sw_data sw_data_;
    robot::support_foot support_foot_;
    mutable std::mutex imu_mtx_, dxl_mtx_, sf_mtx_;
};

#define WM WorldModel::instance()

