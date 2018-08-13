#include "remote.hpp"

using namespace std;

remote::remote(const sub_ptr& s)
{
    attach(s);
    data_.id = 0;
}

bool remote::start()
{
    this->open();
    td_ = thread(bind(&remote::run, this));
    return true;
}

void remote::run()
{
    while(is_alive_)
    {
        if(TCP_SERVER.r_data().id>data_.id)
        {
            data_ = TCP_SERVER.r_data();
            notify();
        }
        usleep(100000);
    }
}

bool remote::open()
{
    is_alive_ = true;
    is_open_ = true;
    return true;
}

void remote::close()
{
    is_alive_ = false;
    is_open_ = false;
}

remote::~remote()
{
    if(td_.joinable()) td_.join();
}
