#ifndef COMM_HPP
#define COMM_HPP

#include <functional>

namespace comm
{
    typedef std::function<void (const char*, const int)> comm_callback;
}
#endif