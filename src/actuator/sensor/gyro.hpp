#ifndef GYRO_HPP
#define GYRO_HPP

#include <list>
#include <memory>
#include "pattern/publisher.hpp"

class gyro: public subject
{
public:
    struct gyro_data
    {
        float pitch;
        float roll;
        float yaw;
    };

    void attach(observer_ptr obs)
    {
        observers_.push_back(obs);
    }

    void detach(observer_ptr obs)
    {
        observers_.remove(obs);
    }
};

#endif