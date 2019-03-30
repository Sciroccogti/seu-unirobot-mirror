#pragma once

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <list>

class task
{
public:
    task()=default;
    task(const std::string &name): name_(name){}

    std::string name() const
    {
        return name_;
    }

    virtual bool perform()
    {
        return true;
    }

private:
    std::string name_;
};

typedef std::shared_ptr<task> task_ptr;
typedef std::list<task_ptr> task_list;
