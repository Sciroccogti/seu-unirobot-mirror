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
using namespace motion;

bool fisrt_lookat = true;
bool kick_complete = true;
/*
std::list<task_ptr> player::play_without_gc()
{
    list<task_ptr> tasks;
    if(WM->fall_data()!=FALL_NONE)
    {
        tasks.push_back(make_shared<look_task>(0.0, -20.0, HEAD_STATE_LOOKAT));
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
            if(dis>0.5)
            {
                fisrt_lookat = true;
                tasks.push_back(make_shared<look_task>(HEAD_STATE_TRACK_BALL));
                if(fabs(dir)>15.0)
                    tasks.push_back(make_shared<walk_task>(0.0, 0.0, dir, true));
                else
                    tasks.push_back(make_shared<walk_task>(0.025, 0.0, 0.0, true));
            }
            else
            {
                double self2left_dir = azimuth(WM->opp_post_left-self.global);
                double self2right_dir = azimuth(WM->opp_post_right-self.global);
                if(self.dir>=self2right_dir && self.dir<=self2left_dir)
                {
                    if(dis>0.2)
                    {
                        if(fabs(dir)>15.0)
                            tasks.push_back(make_shared<walk_task>(0.0, 0.0, dir, true));
                        else
                            tasks.push_back(make_shared<walk_task>(0.015, 0.0, 0.0, true));
                    }
                    else
                    {
                        tasks.push_back(make_shared<look_task>(0.0, 60.0, HEAD_STATE_LOOKAT));
                        if(fisrt_lookat)
                        {
                            fisrt_lookat = false;
                            return tasks;
                        }
                        if(ball.alpha>-0.1)
                            tasks.push_back(make_shared<walk_task>(0.0, -0.012, 0.0, true));
                        else if(ball.alpha<-0.2)
                            tasks.push_back(make_shared<walk_task>(0.0, 0.012, 0.0, true));
                        else
                        {
                            if(ball.beta<0.3)
                                tasks.push_back(make_shared<walk_task>(0.02, 0.0, 0.0, true));
                            else
                            {
                                if(ball.alpha<0)
                                    tasks.push_back(make_shared<action_task>("left_little_kick"));
                                else
                                    tasks.push_back(make_shared<walk_task>(0.0, -0.012, 0.0, true));
                            }
                        }
                    }
                }
                else if(self.dir>self2left_dir)
                {
                    tasks.push_back(make_shared<walk_task>(0.0, 0.015, -10.0, true));
                }
                else
                {
                    tasks.push_back(make_shared<walk_task>(0.0, -0.015, 10.0, true));
                }
            }
        }
        else
        {
            tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));

            //tasks.push_back(make_shared<walk_task>(-0.02, 0.0, 0.0, true));
        }
    }
    return tasks;
}
*/

std::list<task_ptr> player::play_without_gc()
{
    list<task_ptr> tasks;
    if(WM->fall_data()!=FALL_NONE)
    {
        state_ = STATE_GETUP;
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
        {
            double dir = azimuth(ball.self);
            double dis = ball.self.norm();
            //LOG(LOG_INFO)<<dis<<'\t'<<dir<<endll;
            if(dis>0.2)
            {
                fisrt_lookat = true;
                tasks.push_back(make_shared<look_task>(HEAD_STATE_TRACK_BALL));
                if(fabs(dir)>15.0)
                    tasks.push_back(make_shared<walk_task>(0.0, 0.0, dir, true));
                else
                    tasks.push_back(make_shared<walk_task>(0.02, 0.0, 0.0, true));
            }
            else
            {
                //kick_complete = false;
                tasks.push_back(make_shared<look_task>(0.0, 60.0, HEAD_STATE_LOOKAT));
                if(fisrt_lookat)
                {
                    //tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
                    fisrt_lookat = false;
                    return tasks;
                }
                if(ball.alpha>-0.1)
                    tasks.push_back(make_shared<walk_task>(0.0, -0.01, 0.0, true));
                else if(ball.alpha<-0.2)
                    tasks.push_back(make_shared<walk_task>(0.0, 0.01, 0.0, true));
                else
                {
                    if(ball.beta<0.3)
                        tasks.push_back(make_shared<walk_task>(0.015, 0.0, 0.0, true));
                    else
                    {
                        double self2left_dir = azimuth(WM->opp_post_left-self.global);
                        double self2right_dir = azimuth(WM->opp_post_right-self.global);
                        if(self.dir>=self2right_dir && self.dir<=self2left_dir)
                        {
                            //if(ball.alpha<0)
                                tasks.push_back(make_shared<action_task>("left_little_kick"));
                            //else
                            //    tasks.push_back(make_shared<action_task>("right_kick"));
                            //kick_complete = true;
                        }  
                    }
                }
            }
        }
        else
        {
            tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
            //kick_complete = true;
            //tasks.push_back(make_shared<walk_task>(-0.02, 0.0, 0.0, true));
        }
    }
    return tasks;
}