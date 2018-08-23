#ifndef SEU_UNIROBOT_LOGGER_HPP
#define SEU_UNIROBOT_LOGGER_HPP

#include <iostream>
#include "singleton.hpp"

enum log_level
{
    LOG_DEBUG = 1,
    LOG_INFO = 2,
    LOG_WARN = 3,
    LOG_ERROR = 4
};

class logger: public singleton<logger>
{
public:
    logger(): stream(&(std::cout))
    {
        cout_level_ = LOG_INFO;
        curr_level_ = LOG_ERROR;
        enable_ = true;
    }

    logger& operator()(const log_level &l)
    {
        curr_level_ = l;
        return *this;
    }

    template <typename T>
    logger& operator<<(T const &obj)
    {
        if(stream!=nullptr)
        {
            if(enable_ && curr_level_ >= cout_level_)
            *stream<<get_color(curr_level_)<<obj;
        }
        return *this;
    }

    void set_level(const log_level l)
    {
        cout_level_ = l;
    }

    void set_enable(const bool e)
    {
        enable_ = e;
    }
private:
    std::string get_color(const log_level &l)
    {
        std::string clr;
        switch(l)
        {
            case LOG_ERROR:
                clr = "\033[31m";
                break;
            case LOG_WARN:
                clr = "\033[33m";
                break;
            case LOG_INFO:
                clr = "\033[32m";
                break;
            case LOG_DEBUG:
                clr = "\033[35m";
                break;
            default:
                clr = "\033[0m";
                break;
        }
        return clr;
    }

    log_level curr_level_;
    log_level cout_level_;
    bool enable_;
    std::ostream *stream;
};

#define LOG logger::get_singleton()

#endif