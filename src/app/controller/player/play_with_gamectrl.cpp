#include <list>
#include "player.hpp"
#include "plan/action_plan.hpp"
#include "plan/lookat_plan.hpp"
#include "plan/walk_plan.hpp"
#include "tcp.hpp"

using namespace robot;
using namespace std;

list<plan_ptr> player::play_with_gamectrl()
{
    list<plan_ptr> plist;
    RoboCupGameControlData gc_data = WM->gc_data();

    switch (gc_data.state)
    {
        default:
            break;
    }

    return plist;
}
