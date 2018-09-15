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
    //std::cout<<"without gc\n";
    list<plan_ptr> plist;
    //plist = fsm_->run(shared_from_this());
    plist.push_back(make_shared<walk_plan>(0.0, 0.0, 0.0, 0.04));
    //plist.push_back(make_shared<action_plan>("reset"));
    plist.push_back(make_shared<lookat_plan>(0, 45, 100));
    return plist;
}

