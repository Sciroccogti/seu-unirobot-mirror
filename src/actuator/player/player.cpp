#include "player.hpp"
#include "configuration.hpp"
#include "plan/action_plan.hpp"
#include "plan/lookat_plan.hpp"

using namespace std;

player::player(): agent(CONF.get_config_value<int>("think_period"))
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
    LOG<<LOG_DEBUG<<"Thinking..."<<"\n";
    list<plan_ptr> plist;
    if(OPTS.use_remote())
    {
        return play_with_remote();
    }
    plist.push_back(make_shared<action_plan>("ready"));
    plist.push_back(make_shared<lookat_plan>(0,45,100));
    return plist;
}
