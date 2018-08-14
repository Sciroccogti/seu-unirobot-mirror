#ifndef SEU_UNIROBOT_SENSOR_HPP
#define SEU_UNIROBOT_SENSOR_HPP

#include <iostream>
#include <memory>
#include "pattern.hpp"

class sensor: public publisher
{
public:
    sensor(const std::string &name):name_(name)
    {
    }
    ~sensor()
    {
        std::cout<<"\033[32msensor: "<<name_<<" end!\033[0m\n";
    }
    bool is_open() const
    {
        return is_open_;
    }
    
    bool is_alive() const
    {
        return is_alive_;
    }
    
    virtual bool open() = 0;
    virtual bool start() = 0;
    virtual void run() = 0;  
    virtual void stop() = 0;
    
protected:
    bool is_open_;
    bool is_alive_;
    std::string name_;
};

typedef std::shared_ptr<sensor> sensor_ptr;

#endif