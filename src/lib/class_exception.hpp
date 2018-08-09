#ifndef EXCEPTION_HPP
#define EXCEPTION_HPP

#include <iostream>
#include <exception>

template <class CLASS>
class class_exception
{
public:
    class_exception(std::string msg): msg_(msg)
    {
        std::cout<<"\033[31mexception: "<<msg_<<"\033[0m"<<std::endl;
    }

    ~class_exception(){};

    const char *what() const
    {
        return msg_.c_str();
    }

private:
    std::string msg_;
};

#endif