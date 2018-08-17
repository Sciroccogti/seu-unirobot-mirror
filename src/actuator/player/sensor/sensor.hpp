#ifndef SEU_UNIROBOT_SENSOR_HPP
#define SEU_UNIROBOT_SENSOR_HPP

#include <memory>
#include "pattern.hpp"
#include "logger.hpp"
class sensor: public publisher
{
public:
    sensor(const std::string &name):name_(name)
    {
    }

    ~sensor()
    {
        LOG(LOG_INFO, "sensor: [ "+name_+" ] end!");
    }

    virtual bool open() = 0;
    virtual bool start() = 0;
    virtual void run() = 0;  
    virtual void stop() = 0;

    bool is_open() const
    {
        return is_open_;
    }

protected:
    bool is_open_;
    bool is_alive_;
    std::string name_;
};

typedef std::shared_ptr<sensor> sensor_ptr;

#endif
