#ifndef SEU_UNIROBOT_ACTUATOR_ROBOT_SUBSCRIBER_HPP
#define SEU_UNIROBOT_ACTUATOR_ROBOT_SUBSCRIBER_HPP

#include <map>
#include <mutex>
#include "pattern.hpp"
#include "options/options.hpp"
#include "configuration.hpp"
#include "logger.hpp"
#include "sensor/imu.hpp"
#include "motor/rmotor.hpp"
#include "motor/vmotor.hpp"
#include "sensor/gamectrl.hpp"
#include "sensor/server.hpp"

class robot_subscriber: public subscriber
{
public:
    robot_subscriber()
    {
        voltage_ = MAX_VOLTAGE;
    }
    
    bool start()
    {
        sensors_.clear();
        
        if(OPTS.use_debug())
        {
            sensors_["server"] = std::make_shared<server>(shared_from_this());
            if(!sensors_["server"]->start()) return false;
        }
        if(OPTS.use_robot() == options::ROBOT_REAL)
        {
            sensors_["imu"] = std::make_shared<imu>(shared_from_this());
            if(!sensors_["imu"]->start()) return false;
            sensors_["motor"] = std::make_shared<rmotor>(shared_from_this());
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
                LOG(LOG_ERROR, "If you want to use virtual robot, you must run with -d1 -r2");
                return false;
            }
        }
        if(OPTS.use_gc())
        {
            try
            {
                sensors_["gc"] = std::make_shared<gamectrl>(shared_from_this());
                if(!sensors_["gc"]->start()) return false;
            }
            catch(std::exception &e)
            {
                LOG(LOG_WARN, e.what());
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
            return;
        }
        if(pub == (sensors_.find("server"))->second)
        {
            rmt_mtx_.lock();
            std::shared_ptr<server> sptr = std::dynamic_pointer_cast<server>(pub);
            rmt_data_ = sptr->r_data();
            rmt_mtx_.unlock();
            return;
        }
    }
    
    RoboCupGameControlData gc_data() const 
    {
        gc_mtx_.lock();
        RoboCupGameControlData res = gc_data_;
        gc_mtx_.unlock();
        return res;
    }
    
    imu::imu_data imu_data() const 
    { 
        imu_mtx_.lock();
        imu::imu_data res = imu_data_;
        imu_mtx_.unlock();
        return res;
    }
    
    comm::tcp_packet::remote_data rmt_data() const
    {
        rmt_mtx_.lock();
        comm::tcp_packet::remote_data res = rmt_data_;
        rmt_mtx_.unlock();
        return res;
    }

    void reset_rmt_data()
    {
        rmt_mtx_.lock();
        rmt_data_.type = comm::tcp_packet::NONE_DATA;
        rmt_data_.size = 0;
        rmt_mtx_.unlock();
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
