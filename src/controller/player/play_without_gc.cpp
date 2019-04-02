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
        ball_block ball = WM->ball();
        self_block self = WM->self();
        if(ball.can_see)
        {
            double dir = azimuth(ball.self);
            double dis = ball.self.norm();
            //LOG(LOG_INFO)<<dis<<'\t'<<dir<<endll;
            if(dis>0.12)
            {
                if(fabs(dir)>15.0)
                    tasks.push_back(make_shared<walk_task>(0.0, 0.0, -dir, true));
                else
                    tasks.push_back(make_shared<walk_task>(0.027, 0.0, 0.0, true));
            }
            else
            {
                if(fabs(ball.self.y())>0.08)
                    tasks.push_back(make_shared<walk_task>(0.0, sign(ball.self.y())*0.02, 0.0, true));
                else
                {
                    if(ball.self.x()>0.05)
                        tasks.push_back(make_shared<walk_task>(0.02, 0.0, 0.0, true));
                    else
                    {
                        if(ball.self.y()>0)
                        {
                            tasks.push_back(make_shared<action_task>("left_kick"));
                            //tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
                            //LOG(LOG_WARN)<<"left_kick"<<endll;
                        }
                        else
                        {
                            tasks.push_back(make_shared<action_task>("right_kick"));
                            //tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
                            //LOG(LOG_WARN)<<"right_kick"<<endll;
                        }
                    }
                }
            }
        }
        else
        {
            tasks.push_back(make_shared<walk_task>(-0.02, 0.0, 0.0, true));
        }
    }
    return tasks;
}