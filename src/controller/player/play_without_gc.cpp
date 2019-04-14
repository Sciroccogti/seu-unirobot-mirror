#include <list>
#include "player.hpp"
#include "task/look_task.hpp"
#include "task/walk_task.hpp"
#include "task/action_task.hpp"
#include "core/worldmodel.hpp"
#include "configuration.hpp"
#include "math/math.hpp"
#include "vision/vision.hpp"

using namespace std;
using namespace robot;
using namespace Eigen;
using namespace robot_math;
using namespace motion;


std::list<task_ptr> player::play_without_gc()
{
    list<task_ptr> tasks;
    if(WM->fall_data()!=FALL_NONE)
    {
        tasks.push_back(make_shared<look_task>(0.0, 0.0, HEAD_STATE_LOOKAT));
        if(WM->fall_data()==FALL_FORWARD)
            tasks.push_back(make_shared<action_task>("front_getup"));
        else if(WM->fall_data()==FALL_FORWARD)
            tasks.push_back(make_shared<action_task>("back_getup"));
    }
    else
    {
        ball_block ball = WM->ball();
        self_block self = WM->self();
        if(ball.can_see)
            tasks = play_skill_kick(self, ball);
        else
            tasks = play_skill_search_ball();
    }
    return tasks;
}