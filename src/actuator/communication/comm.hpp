#ifndef COMM_HPP
#define COMM_HPP

#include <boost/thread.hpp>

namespace communication
{
    class comm
    {
    public:
        comm():is_open_(false)
        {
        }
        virtual bool open()=0;
        virtual bool close()=0;
        virtual bool read(std::string &data, int &size)=0;
        virtual bool write(const std::string &data)=0;
    protected:
        boost::mutex read_mutex_;
        boost::mutex write_mutex_;
        bool is_open_;
    };
}

#endif