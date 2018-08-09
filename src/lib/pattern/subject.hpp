#ifndef SUBJECT_HPP
#define SUBJECT_HPP

#include "observer.hpp"

class subject
{
public:
    virtual void attach(observer_ptr) = 0;
    virtual void detach(observer_ptr) = 0;
    virtual void notify(const char*, const int&) = 0;
};

#endif