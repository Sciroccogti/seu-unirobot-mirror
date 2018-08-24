#include <list>
#include "player.hpp"
#include "plan/action_plan.hpp"
#include "plan/lookat_plan.hpp"
#include "plan/walk_plan.hpp"
#include "tcp.hpp"

using namespace robot;
using namespace std;

list<plan_ptr> player::play_without_gamectrl()
{
    list<plan_ptr> plist;
    plist.push_back(make_shared<action_plan>("ready"));
    plist.push_back(make_shared<lookat_plan>(0,45,100));
    return plist;
}

