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
        LOG(LOG_DEBUG)<<"sensor: [ "+name_+" ] construct\n";
    }

    ~sensor()
    {
        LOG(LOG_INFO)<<"sensor: [ "+name_+" ] end!\n";
    }

    virtual bool start() = 0;
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
