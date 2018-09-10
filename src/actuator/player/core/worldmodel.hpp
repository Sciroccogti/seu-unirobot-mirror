#ifndef SEU_UNIROBOT_ACTUATOR_WORLD_MODEL_HPP
#define SEU_UNIROBOT_ACTUATOR_WORLD_MODEL_HPP

#include <mutex>
#include "pattern.hpp"
#include "sensor/imu.hpp"
#include "sensor/motor.hpp"
#include "sensor/gamectrl.hpp"
#include "sensor/server.hpp"
#include "sensor/hear.hpp"
#include "configuration.hpp"

class worldmodel: public subscriber
{
public:
    worldmodel()
    {
        rmt_data_.type = NON_DATA;
        rmt_data_.size = 0;
        low_power_ = false;
        is_lost_ = false;
    }

    void updata(const pro_ptr &pub, const int &type)
    {
        if(type == sensor::SENSOR_GC)
        {
            gc_mtx_.lock();
            std::shared_ptr<gamectrl> sptr = std::dynamic_pointer_cast<gamectrl>(pub);
            gc_data_ = sptr->data();
            gc_mtx_.unlock();
            std::cout<<(int)gc_data_.state<<std::endl;
            return;
        }
        if(type == sensor::SENSOR_HEAR)
        {
            hear_mtx_.lock();
            std::shared_ptr<hear> sptr = std::dynamic_pointer_cast<hear>(pub);
            players_[sptr->info().id] = sptr->info();
            hear_mtx_.unlock();
            return;
        }
        if(type == sensor::SENSOR_IMU)
        {
            imu_mtx_.lock();
            std::shared_ptr<imu> sptr = std::dynamic_pointer_cast<imu>(pub);
            imu_data_ = sptr->data();
            imu_mtx_.unlock();
            return;
        }
        if(type == sensor::SENSOR_MOTOR)
        {
            dxl_mtx_.lock();
            std::shared_ptr<motor> sptr = std::dynamic_pointer_cast<motor>(pub);
            low_power_ = sptr->low_power();
            is_lost_ = sptr->is_lost();
            dxl_mtx_.unlock();
            return;
        }
        if(type == sensor::SENSOR_SERVER)
        {
            rmt_mtx_.lock();
            std::shared_ptr<tcp_server> sptr = std::dynamic_pointer_cast<tcp_server>(pub);
            rmt_data_ = sptr->r_data();
            rmt_mtx_.unlock();
            return;
        }
    }

    RoboCupGameControlData gc_data() const
    {
        std::lock_guard<std::mutex> lk(gc_mtx_);
        return gc_data_;
    }

    imu::imu_data imu_data() const
    {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        return imu_data_;
    }

    std::map<int, player_info> players() const
    {
        std::lock_guard<std::mutex> lk(hear_mtx_);
        return players_;
    }

    remote_data rmt_data() const
    {
        std::lock_guard<std::mutex> lk(rmt_mtx_);
        return rmt_data_;
    }

    void reset_rmt_data()
    {
        std::lock_guard<std::mutex> lk(rmt_mtx_);
        rmt_data_.type = NON_DATA;
        rmt_data_.size = 0;
    }

    bool low_power() const { return low_power_; }
    bool is_lost() const { return is_lost_; }

private:
    bool low_power_, is_lost_;
    std::map<int, player_info> players_;
    RoboCupGameControlData gc_data_;
    imu::imu_data imu_data_;
    remote_data rmt_data_;
    mutable std::mutex gc_mtx_, imu_mtx_, rmt_mtx_, dxl_mtx_, hear_mtx_;
};

#endif
