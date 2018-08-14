#include "remote.hpp"

using namespace std;

remote::remote(const sub_ptr& s): sensor("remote")
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
        cout<<"run\n";
        std::lock_guard<std::mutex> lk(TCP_SERVER.tcp_mutex_);
        if(TCP_SERVER.r_data().id>data_.id)
        {
            data_ = TCP_SERVER.r_data();
            notify();
        }
        cout<<"run\n";
        usleep(100000);
    }
    cout<<"run end\n";
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
    std::cout<<"\033[32mremote closed!\033[0m\n";
}

remote::~remote()
{
    if(td_.joinable()) td_.join();
}
