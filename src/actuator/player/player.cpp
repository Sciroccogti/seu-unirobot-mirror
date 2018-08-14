#include "player.hpp"
#include "configuration.hpp"
#include "plan/action_plan.hpp"
#include "plan/test_plan.hpp"

using namespace std;

player::player(): agent(CONF.get_config_value<int>("period"))
{
}

bool player::initialization()
{
    if(!init()) return false;
    start_timer();
    return true;
}

list< plan_ptr > player::think()
{     
    list<plan_ptr> plist;
    plist.push_back(make_shared<action_plan>("ready", suber_->get_sensor("motor")));
    return plist;
}
