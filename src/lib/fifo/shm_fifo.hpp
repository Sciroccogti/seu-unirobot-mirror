#ifndef SHM_FIFO_HPP
#define SHM_FIFO_HPP

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <string>

namespace fifo
{
    using namespace boost::interprocess;

    class shm_fifo
    {
        shared_memory_object share_mem_;
        mapped_region mapped_reg_;
        std::string name_;
        int blocks_;
        int blksz_;
        int size_;
        unsigned int rd_idx_;
        unsigned int wr_idx_;

    public:
        shm_fifo(std::string name, int blksz, int blocks): name_(name), blocks_(blocks), blksz_(blksz)
        {
            share_mem_ = shared_memory_object(open_or_create, name_.c_str(), read_write);
            size_ = blocks_*blksz_;
            share_mem_.truncate(size_);
            mapped_reg_ = mapped_region(share_mem_, read_write);
            rd_idx_ = 0;
            wr_idx_ = 0;
        }

        bool get(char *data)
        {
            if(data == nullptr) return false;
            data = static_cast<char*>(mapped_reg_.get_address()+size_*(rd_idx_%blocks_));
            memset(mapped_reg_.get_address()+size_*(rd_idx_%blocks_), 0, blksz_);
            rd_idx_++;
            return true;
        }

        bool put(const char *data)
        {
            if(data == nullptr) return false;
            memcpy(static_cast<char*>(mapped_reg_.get_address()+size_*(wr_idx_++%blocks_)), data, blksz_);
            return true;
        }

        ~shm_fifo()
        {
            shared_memory_object::remove(name_.c_str());
        }
    };
}

#endif