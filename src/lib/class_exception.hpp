#pragma once

#include <exception>
#include <iostream>
#include "logger.hpp"

template <class CLASS>
class class_exception
{
public:
    class_exception(std::string msg, int id = 0): msg_(msg), id_(id)
    {
        LOG(LOG_WARN) << "exception: " + msg_ << endll;
    }

    ~class_exception() {};

    inline char *what() const
    {
        return msg_.c_str();
    }

    inline int err_no() const
    {
        return id_;
    }
private:
    std::string msg_;
    int id_;
};
