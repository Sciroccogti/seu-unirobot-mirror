#ifndef SEU_UNIROBOT_OBSERVER_HPP
#define SEU_UNIROBOT_OBSERVER_HPP

#include "publisher.hpp"

class subscriber: public std::enable_shared_from_this<subscriber>
{
public:
    virtual void updata(const sub_ptr &sub) = 0;
};

#endif