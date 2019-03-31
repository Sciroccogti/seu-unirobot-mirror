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
                Vector2i pix(bb.pixel.x()-w_/2, h_-bb.pixel.y());
                if(pix.y()<1) pix.y()=1;
                float yaw=rad2deg(atan(pix.x()/(float)pix.y()));
                std::vector<double> head_degs = ROBOT->get_head_degs();
                LOG(LOG_INFO)<<yaw<<endll;
                //tasks.push_back(make_shared<look_task>(head_degs[0]-yaw, head_degs[1], false));//+sign(pix.y()-h_/2)*0.05*std::abs(pix.y()-h_/2)
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