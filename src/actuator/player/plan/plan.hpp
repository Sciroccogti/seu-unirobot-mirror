#ifndef SEU_UNIROBOT_PLAN_HPP
#define SEU_UNIROBOT_PLAN_HPP

#include <memory>

class plan
{
public:
    plan(const std::string &pname): name_(pname)
    {
    }
    
    virtual bool perform()
    {
        return true;
    }
    
    std::string name() const
    {
        return name_;
    }
protected:
    std::string name_;
};

typedef std::shared_ptr<plan> plan_ptr;

#endif