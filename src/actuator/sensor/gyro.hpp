#ifndef GYRO_HPP
#define GYRO_HPP

#include <list>
#include <memory>
#include "pattern/subject.hpp"

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

    void notify(const char* data, const int &size)
    {
        for(auto obs: observers_)
            obs->updata(data, size);
    }
    void notify()
    {
        //for(auto obs: observers_)
        //    obs->updata();
    }
private:
    std::list< observer_ptr > observers_;
};

#endif