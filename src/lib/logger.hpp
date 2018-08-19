#ifndef SEU_UNIROBOT_LOGGER_HPP
#define SEU_UNIROBOT_LOGGER_HPP

#include <iostream>

enum log_level
{
    LOG_DEFAULT = 0,
    LOG_ERROR = 1,
    LOG_WARN = 2,
    LOG_INFO = 3
};
    
static std::string get_color(const log_level &ll)
{
    std::string str;
    switch(ll)
    {
        case LOG_ERROR:
            str = "\033[31m";
            break;
        case LOG_WARN:
            str = "\033[33m";
            break;
        case LOG_INFO:
            str = "\033[32m";
            break;
        default:
            str = "\033[0m";
            break;
    }
    return str;
}

#define LOG(level, value) std::cout<<get_color(level)<<value<<"\033[0m"<<std::endl;

#endif