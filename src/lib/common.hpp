#ifndef __COMMON_HPP
#define __COMMON_HPP

#include <ctime>
#include <cstdio>
#include <string>
#include <iomanip>

inline std::string get_time()
{
    time_t timep;
    std::time(&timep);
    char tmp[64];
    std::strftime(tmp, sizeof(tmp), "%Y-%m-%d_%H%M%S", std::localtime(&timep));
    return std::string(tmp);
}

#endif
