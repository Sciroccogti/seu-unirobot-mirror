#ifndef SEU_UNIROBOT_SENSOR_HPP
#define SEU_UNIROBOT_SENSOR_HPP

#include <memory>
#include "pattern.hpp"

class sensor: public publisher
{
public:
    bool is_open() const
    {
        return is_open_;
    }
    
    bool is_alive() const
    {
        return is_alive_;
    }
    
    virtual bool start() = 0;
    virtual void run() = 0;  
    virtual bool open() = 0;
    virtual void close() = 0;
    
protected:
    bool is_open_;
    bool is_alive_;
};

typedef std::shared_ptr<sensor> sersor_ptr;

#endif