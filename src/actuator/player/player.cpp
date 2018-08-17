#include "player.hpp"
#include "configuration.hpp"
#include "plan/action_plan.hpp"
#include "plan/lookat_plan.hpp"

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
    //LOG(LOG_INFO, "Thinking...");
    list<plan_ptr> plist;
    if(OPTS.use_remote())
    {
        return play_with_remote();
    }
    //plist.push_back(make_shared<action_plan>("ready", suber_->get_sensor("motor")));
    //plist.push_back(make_shared<lookat_plan>(90,90,suber_->get_sensor("motor"), 100));
    return plist;
}
