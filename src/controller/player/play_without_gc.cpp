#include <list>
#include "player.hpp"
#include "task/look_task.hpp"
#include "task/walk_task.hpp"
#include "task/action_task.hpp"
#include "core/worldmodel.hpp"
#include "configuration.hpp"
#include "math/math.hpp"
#include "vision/vision.hpp"
#include "skill.hpp"

using namespace std;
using namespace robot;
using namespace Eigen;
using namespace robot_math;

std::list<task_ptr> player::play_without_gc()
{
    list<task_ptr> tasks;
    if(WM->fall_data()!=FALL_NONE)
    {
        state_ = STATE_GETUP;
        tasks.push_back(make_shared<look_task>(false));
        if(WM->fall_data()==FALL_FORWARD)
            tasks.push_back(make_shared<action_task>("front_getup"));
        else if(WM->fall_data()==FALL_FORWARD)
            tasks.push_back(make_shared<action_task>("back_getup"));
    }
    else
    {
        ball_block bb = WM->ball();
        if(bb.sure)
        {
            double d=bb.self.norm();
            //if(d<0.5)
            {
                state_=STATE_KICK;
                tasks.push_back(skill_track_ball());
            }
            /*
            else
            {
                state_=STATE_SEARCH;
                tasks.push_back(make_shared<look_task>(true));
                tasks.push_back(skill_goto(bb.global, 0, false));
            } */
        }
    }
    return tasks;
}