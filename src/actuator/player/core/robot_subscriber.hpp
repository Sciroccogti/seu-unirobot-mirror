#ifndef SEU_UNIROBOT_ROBOT_SUBSCRIBER_HPP
#define SEU_UNIROBOT_ROBOT_SUBSCRIBER_HPP

#include <map>
#include <mutex>
#include "pattern.hpp"
#include "options/options.hpp"
#include "configuration.hpp"
#include "../sensor/imu.hpp"
#include "../sensor/remote.hpp"
#include "../sensor/motor.hpp"
#include "../sensor/game_ctrl.hpp"

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
        sensors_["motor"] = std::make_shared<motor>(shared_from_this());
        if(OPTS.use_debug())
        {
            if(OPTS.use_remote()) sensors_["remote"] = std::make_shared<remote>(shared_from_this());
        }
        if(OPTS.use_robot())
        {
            sensors_["imu"] = std::make_shared<imu>(shared_from_this());
        }
        if(OPTS.use_gc())
        {
            try
            {
                sensors_["gc"] = std::make_shared<game_ctrl>(shared_from_this());
            }
            catch(std::exception &e)
            {
                std::cout<<"\033[31m"<<e.what()<<"\033[0m\n";
            }
        }
        auto iter = sensors_.begin();
        while(iter!=sensors_.end())
        {
            if(!(iter->second->start())) return false;
            iter++;
        }
        return true;
    }
    
    void unregist()
    {
        auto iter = sensors_.begin();
        while(iter!=sensors_.end())
        {
            iter->second->detach(shared_from_this());
            iter->second->close();
            iter++;
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
        //std::cout<<"update\n";
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
        if(pub == (sensors_.find("remote"))->second)
        {
            //std::cout<<"update rmt\n";
            std::lock_guard<std::mutex> lk(rmt_mtx_);
            std::shared_ptr<remote> sptr = std::dynamic_pointer_cast<remote>(pub);
            rmt_data_ = sptr->data();
            return;
        }
        if(pub == (sensors_.find("motor"))->second)
        {
            std::lock_guard<std::mutex> lk(dxl_mtx_);
            std::shared_ptr<motor> sptr = std::dynamic_pointer_cast<motor>(pub);
            voltage_ = sptr->voltage();
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

private:
    float voltage_;
    std::map<std::string, sensor_ptr> sensors_;
    RoboCupGameControlData gc_data_;
    imu::imu_data imu_data_;
    comm::tcp_packet::remote_data rmt_data_;
    mutable std::mutex gc_mtx_, imu_mtx_, rmt_mtx_, dxl_mtx_;
};

#endif //SEU_UNIROBOT_ROBOT_SUBSCRIBER_HPP
