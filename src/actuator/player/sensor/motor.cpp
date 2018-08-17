#include "motor.hpp"
#include "configuration.hpp"

using namespace std;
using namespace robot;

motor::motor(): timer(CONF.get_config_value<int>("serial.dxl.period")), sensor("motor"),
    period_(CONF.get_config_value<int>("serial.dxl.period")), p_count_(0)
{
}

bool motor::start()
{
    if(!open()) return false;
    sensor::is_alive_ = true;
    timer::is_alive_ = true;
    start_timer();
    return true;
}

void motor::run()
{
    std::map<int, float> jdmap;
    if(timer::is_alive_)
    {
        {
            lock_guard<mutex> lk(bd_mutex_);
            if(!body_degs_list.empty())
            {
                jdmap = body_degs_list.front();
                body_degs_list.pop_front();
                ROBOT.set_degs(jdmap);
            }
        }
        {
            lock_guard<mutex> lk(hd_mutex_);
            if(!head_degs_list.empty())
            {
                jdmap = head_degs_list.front();
                head_degs_list.pop_front();
                ROBOT.set_degs(jdmap);
            }
        }
        act();
        p_count_++;
    }
}

void motor::stop()
{
    sensor::is_alive_ = false;
    timer::is_alive_ = false;
    close();
    delete_timer();
    sleep(1);
    body_degs_list.clear();
    head_degs_list.clear();
}


bool motor::add_body_degs(const map<int, float>& jdmap)
{
    if(!sensor::is_alive_) return false;
    lock_guard<mutex> lk(bd_mutex_);
    body_degs_list.push_back(jdmap);
    return true;
}

bool motor::add_head_degs(const map<int, float>& jdmap)
{
    if(!sensor::is_alive_) return false;
    lock_guard<mutex> lk(hd_mutex_);
    head_degs_list.push_back(jdmap);
    return true;
}

motor::~motor()
{

}
