#ifndef EXCEPTION_HPP
#define EXCEPTION_HPP

#include <iostream>
#include <exception>

template <class CLASS>
class class_exception
{
public:
    class_exception(std::string msg, int id=0): msg_(msg), id_(id)
    {
        std::cout<<"\033[31mexception: "<<msg_<<"\033[0m"<<std::endl;
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