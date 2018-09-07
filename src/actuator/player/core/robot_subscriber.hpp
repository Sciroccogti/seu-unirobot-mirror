#ifndef SEU_UNIROBOT_ACTUATOR_ROBOT_SUBSCRIBER_HPP
#define SEU_UNIROBOT_ACTUATOR_ROBOT_SUBSCRIBER_HPP

#include <map>
#include <mutex>
#include "pattern.hpp"
#include "options/options.hpp"
#include "configuration.hpp"
#include "sensor/imu.hpp"
#include "motor/rmotor.hpp"
#include "motor/vmotor.hpp"
#include "sensor/gamectrl.hpp"
#include "sensor/tcp_server.hpp"
#include "sensor/hear.hpp"

class robot_subscriber: public subscriber
{
public:
    robot_subscriber()
    {
        voltage_ = MAX_VOLTAGE;
        rmt_data_.type = NON_DATA;
        rmt_data_.size = 0;
    }
    
    bool start()
    {
        sensors_.clear();
        if(OPTS.use_debug())
        {
            sensors_["server"] = std::make_shared<tcp_server>(shared_from_this());
            sensors_["server"]->start();
        }
        if(OPTS.use_robot() == options::ROBOT_REAL)
        {
            //sensors_["imu"] = std::make_shared<imu>(shared_from_this());
            //if(!sensors_["imu"]->start()) return false;
            sensors_["motor"] = std::make_shared<rmotor>(shared_from_this());
            if(!sensors_["motor"]->start()) return false;
        }
        else if(OPTS.use_robot() == options::ROBOT_VIRTUAL)
        {
            if(OPTS.use_debug())
            {
                sensors_["motor"] = std::make_shared<vmotor>(sensors_["server"]);
                if(!sensors_["motor"]->start()) return false;
            }
            else
            {
                std::cout<<"If you want to use virtual robot, you must run with -d1 -r2 \n";
                return false;
            }
        }
        if(OPTS.use_gc())
        {
            try
            {
                sensors_["gc"] = std::make_shared<gamectrl>(shared_from_this());
                sensors_["gc"]->start();
            }
            catch(std::exception &e)
            {
                std::cout<<e.what()<<"\n";
            }
        }
        if(OPTS.use_comm())
        {
            try
            {
                sensors_["hear"] = std::make_shared<hear>(shared_from_this());
                sensors_["hear"]->start();
            }
            catch(std::exception &e)
            {
                std::cout<<e.what()<<"\n";
            }
        }
        return true;
    }
    
    void stop()
    {
        if(sensors_.find("gc") != sensors_.end())
        {
            sensors_["gc"]->detach(shared_from_this());
            sensors_["gc"]->stop();
        }
        if(sensors_.find("hear") != sensors_.end())
        {
            sensors_["hear"]->detach(shared_from_this());
            sensors_["hear"]->stop();
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
        }
        if(sensors_.find("server") != sensors_.end())
        {
            sensors_["server"]->detach(shared_from_this());
            sensors_["server"]->stop();
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
            gc_mtx_.lock();
            std::shared_ptr<gamectrl> sptr = std::dynamic_pointer_cast<gamectrl>(pub);
            gc_data_ = sptr->data();
            gc_mtx_.unlock();
            std::cout<<(int)gc_data_.state<<std::endl;
            return;
        }
        if(pub == (sensors_.find("hear"))->second)
        {
            hear_mtx_.lock();
            std::shared_ptr<hear> sptr = std::dynamic_pointer_cast<hear>(pub);
            players_[sptr->info().id] = sptr->info();
            hear_mtx_.unlock();
            return;
        }
        if(pub == (sensors_.find("imu"))->second)
        {
            imu_mtx_.lock();
            std::shared_ptr<imu> sptr = std::dynamic_pointer_cast<imu>(pub);
            imu_data_ = sptr->data();
            imu_mtx_.unlock();
            return;
        }
        if(pub == (sensors_.find("motor"))->second)
        {
            dxl_mtx_.lock();
            std::shared_ptr<rmotor> sptr = std::dynamic_pointer_cast<rmotor>(pub);
            voltage_ = sptr->voltage();
            dxl_mtx_.unlock();
            //std::cout<<voltage_<<std::endl;
            return;
        }
        if(pub == (sensors_.find("server"))->second)
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

private:
    float voltage_;
    std::map<int, player_info> players_;
    std::map<std::string, sensor_ptr> sensors_;
    RoboCupGameControlData gc_data_;
    imu::imu_data imu_data_;
    remote_data rmt_data_;
    mutable std::mutex gc_mtx_, imu_mtx_, rmt_mtx_, dxl_mtx_, hear_mtx_;
};

#endif //SEU_UNIROBOT_ROBOT_SUBSCRIBER_HPP
