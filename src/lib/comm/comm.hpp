#ifndef SEU_UNIROBOT_COMM_HPP
#define SEU_UNIROBOT_COMM_HPP

#include <functional>

namespace comm
{
    typedef std::function<void (const char*, const int&, const int&)> net_comm_callback;
    typedef std::function<void (const char*, const int&)> ser_comm_callback;
}
#endif