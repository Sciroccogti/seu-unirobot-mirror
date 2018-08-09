#ifndef OBSERVER_HPP
#define OBSERVER_HPP

#include <memory>

class observer
{
public:
    virtual void updata(const char*, const int&) = 0;
};

typedef std::shared_ptr<observer> observer_ptr;

#endif