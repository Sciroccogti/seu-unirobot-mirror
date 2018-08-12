#ifndef SEU_UNIROBOT_ROBOT_SUBSCRIBER_HPP
#define SEU_UNIROBOT_ROBOT_SUBSCRIBER_HPP

#include "pattern/subscriber.hpp"
#include "options/options.hpp"
#include "configuration.hpp"

class robot_subscriber: public subscriber
{
public:
    bool regist()
    {

    }

    void update(const sub_ptr &sub)
    {

    }

public:
    sub_ptr imu_;
};

#endif //SEU_UNIROBOT_ROBOT_SUBSCRIBER_HPP
