#ifndef PIPE_FIFO_HPP
#define PIPE_FIFO_HPP

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <string>

namespace fifo
{
    enum pipe_mode
    {
        RDONLY = O_RDONLY,
        WRONLY = O_WRONLY
    };

    class pipe_fifo
    {
        int fd_;
        pipe_mode mode_;
        std::string path_;
    public:
        pipe_fifo(std::string path, pipe_mode mode)
        {
            this->path_=path; 
            this->mode_=mode;
        }

        bool p_open()
        {
            if(access(path_.c_str(), F_OK)==0) remove(path_.c_str());
            if(mkfifo(path_.c_str(), 0777)!=0) return false;
            fd_ = open(path_.c_str(), mode_);
            if(fd_ == -1) return false;
            return true;
        }

        bool p_write(const char *data, int size)
        {
            if(mode_ == pipe_mode::RDONLY) return false;
            if(write(fd_, data, size)==-1) return false;
            return true;
        }

        bool p_read(char *data, int size)
        {
            if(mode_ == pipe_mode::WRONLY) return false;
            if(read(fd_, data, size)==-1) return false;
            return true;
        }

        bool p_close()
        {
            if(close(fd_)==-1) return false;
            return true;
        }
    };
}

#endif

