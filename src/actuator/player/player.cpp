#include "player.hpp"
#include "configuration.hpp"
#include "plan/action_plan.hpp"
#include "plan/lookat_plan.hpp"
#include "plan/say_plan.hpp"

using namespace std;
using namespace FSM;
using namespace walk;

player::player(): agent(CONF.get_config_value<int>("think_period"))
{
}

bool player::initialization()
{
    if(!init()) return false;
    fsm_ = make_shared<fsm>();
    start_timer();
    WALK.start(suber_->get_sensor("motor"));
    return true;
}

list< plan_ptr > player::think()
{
    list<plan_ptr> plist;
    list<plan_ptr> tlist;
    if(OPTS.use_remote())
    {
        tlist = play_with_remote();
    }
    else
    {
        if(OPTS.use_gc())
            tlist = play_with_gamectrl();
        else
            tlist = play_without_gamectrl();
    }
    plist.insert(plist.end(), tlist.begin(), tlist.end());
    return plist;
}
