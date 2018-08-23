#ifndef SEU_UNIROBOT_EXCEPTION_HPP
#define SEU_UNIROBOT_EXCEPTION_HPP

#include <exception>
#include "logger.hpp"

template <class CLASS>
class class_exception
{
public:
    class_exception(std::string msg, int id=0): msg_(msg), id_(id)
    {
        LOG(LOG_ERROR)<<"exception: "+msg_<<"\n";
    }

    ~class_exception(){};

    const char *what() const
    {
        return msg_.c_str();
    }

    int err_no() const
    {
        return id_;
    }
private:
    std::string msg_;
    int id_;
};

#endif