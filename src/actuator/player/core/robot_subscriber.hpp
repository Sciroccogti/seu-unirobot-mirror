#ifndef SEU_UNIROBOT_ACTUATOR_ROBOT_SUBSCRIBER_HPP
#define SEU_UNIROBOT_ACTUATOR_ROBOT_SUBSCRIBER_HPP

#include <map>
#include <mutex>
#include "pattern.hpp"
#include "options/options.hpp"
#include "configuration.hpp"
#include "logger.hpp"
#include "sensor/imu.hpp"
#include "sensor/rmotor.hpp"
#include "sensor/vmotor.hpp"
#include "sensor/game_ctrl.hpp"
#include "sensor/debuger.hpp"

class robot_subscriber: public subscriber
{
public:
    robot_subscriber()
    {
        voltage_ = MAX_VOLTAGE;
    }
    
    bool regist()
    {
        sensors_.clear();
        if(OPTS.use_debug())
        {
            sensors_["debug"] = std::make_shared<debuger>(shared_from_this());
            if(!sensors_["debug"]->start()) return false;
        }
        if(OPTS.use_robot())
        {
            sensors_["imu"] = std::make_shared<imu>(shared_from_this());
            if(!sensors_["imu"]->start()) return false;
            sensors_["motor"] = std::make_shared<rmotor>(shared_from_this());
        }
        else
        {
            if(OPTS.use_debug())
            {
                sensors_["motor"] = std::make_shared<vmotor>(shared_from_this(), sensors_["debug"]);
                if(!sensors_["motor"]->start()) return false;
            }
        }
        if(OPTS.use_gc())
        {
            try
            {
                sensors_["gc"] = std::make_shared<game_ctrl>(shared_from_this());
                if(!sensors_["gc"]->start()) return false;
            }
            catch(std::exception &e)
            {
                LOG(LOG_WARN, e.what());
            }
        }
        return true;
    }
    
    void unregist()
    {
        if(sensors_.find("gc") != sensors_.end())
        {
            sensors_["gc"]->detach(shared_from_this());
            sensors_["gc"]->stop();
        }
        if(sensors_.find("imu") != sensors_.end())
        {
            sensors_["imu"]->detach(shared_from_this());
            sensors_["imu"]->stop();
        }
        if(sensors_.find("motor") != sensors_.end())
        {
            sensors_["motor"]->detach(shared_from_this());
            sensors_["motor"]->stop();
            //sensors_["motor"]
        }
        if(sensors_.find("debug") != sensors_.end())
        {
            sensors_["debug"]->detach(shared_from_this());
            sensors_["debug"]->stop();
        }
    }
    
    sensor_ptr get_sensor(const std::string &name)
    {
        auto iter = sensors_.find(name);
        if(iter != sensors_.end())
            return iter->second;
        return nullptr;
    }
    
    virtual void updata(const pub_ptr &pub)
    {
        if(pub == (sensors_.find("gc"))->second)
        {
            std::lock_guard<std::mutex> lk(gc_mtx_);
            std::shared_ptr<game_ctrl> sptr = std::dynamic_pointer_cast<game_ctrl>(pub);
            gc_data_ = sptr->data();
            return;
        }
        if(pub == (sensors_.find("imu"))->second)
        {
            std::lock_guard<std::mutex> lk(imu_mtx_);
            std::shared_ptr<imu> sptr = std::dynamic_pointer_cast<imu>(pub);
            imu_data_ = sptr->data();
            return;
        }
        if(pub == (sensors_.find("motor"))->second)
        {
            std::lock_guard<std::mutex> lk(dxl_mtx_);
            std::shared_ptr<rmotor> sptr = std::dynamic_pointer_cast<rmotor>(pub);
            voltage_ = sptr->voltage();
            return;
        }
        if(pub == (sensors_.find("debug"))->second)
        {
            std::lock_guard<std::mutex> lk(rmt_mtx_);
            std::shared_ptr<debuger> sptr = std::dynamic_pointer_cast<debuger>(pub);
            rmt_data_ = sptr->r_data();
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
    
    comm::tcp_packet::remote_data rmt_data() const
    {
        std::lock_guard<std::mutex> lk(rmt_mtx_);
        return rmt_data_;
    }

    void reset_rmt_data()
    {
        std::lock_guard<std::mutex> lk(rmt_mtx_);
        rmt_data_.type = comm::tcp_packet::NONE_DATA;
        rmt_data_.size = 0;
    }

private:
    float voltage_;
    std::map<std::string, sensor_ptr> sensors_;
    RoboCupGameControlData gc_data_;
    imu::imu_data imu_data_;
    comm::tcp_packet::remote_data rmt_data_;
    mutable std::mutex gc_mtx_, imu_mtx_, rmt_mtx_, dxl_mtx_;
};

#endif //SEU_UNIROBOT_ROBOT_SUBSCRIBER_HPP
