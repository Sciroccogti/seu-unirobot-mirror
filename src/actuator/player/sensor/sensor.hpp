#ifndef SEU_UNIROBOT_SENSOR_HPP
#define SEU_UNIROBOT_SENSOR_HPP

#include <iomanip>
#include <memory>
#include "pattern.hpp"
#include <iostream>

class sensor: public promulgator
{
public:
    enum sensor_type
    {
        SENSOR_NONE = 0,
        SENSOR_SERVER = 1,
        SENSOR_GC = 2,
        SENSOR_HEAR = 3,
        SENSOR_IMU = 4,
        SENSOR_MOTOR = 5,
        SENSOR_CAMERA = 6
    };
    sensor(const std::string &name): name_(name)
    {
    }

    ~sensor()
    {
        std::cout << std::setw(15) << "\033[32msensor: " << std::setw(10) << "[" + name_ + "]" << " end!\n\033[0m";
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
