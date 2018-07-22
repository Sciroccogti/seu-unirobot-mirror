#ifndef UDP_PACKET_HPP
#define UDP_PACKET_HPP

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "class_exception.hpp"

namespace comm
{
    class udp_packet
    {
    public:
        enum {max_data_length = 1024};
        
        udp_packet():data_length_(0)
        {
        }
        
        udp_packet(const char* data, const int &size)
        {
            if(size>max_data_length)
            {
                data_length_ = 0;
                throw class_exception<udp_packet>("the cmd size is larger than the max length");
            }
            else
            {
                data_length_ = size;
                std::memcpy(data_, data, size);
            }
        }
        
        char *data()
        {
            return data_;
        }
        
        const char *data() const
        {
            return data_;
        }
        
        int length() const
        {
            return data_length_;
        }
        
    private:
        char data_[max_data_length];
        int data_length_;
    };
}
#endif