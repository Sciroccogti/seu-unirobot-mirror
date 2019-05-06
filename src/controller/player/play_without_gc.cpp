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
    self_location_count_++;
    if(WM->fall_data()!=FALL_NONE)
    {
        if(WM->fall_data()==FALL_FORWARD)
            tasks.push_back(make_shared<action_task>("front_getup"));
        else if(WM->fall_data()==FALL_BACKWARD)
            tasks.push_back(make_shared<action_task>("back_getup"));
        else if(WM->fall_data()==FALL_LEFT)
            tasks.push_back(make_shared<action_task>("right_arm"));
        else
            tasks.push_back(make_shared<action_task>("left_arm"));
    }
    else
    {
        if(CONF->get_my_role()=="keeper")
        {
            ball_block ball = WM->ball();
            self_block self = WM->self();
            if(!ball.can_see)
            {
                if(keeper_kicked_)
                    tasks.push_back(play_skill_goto(Vector2d(-4.5, 0.0), 0.0));
                else
                {
                    tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
                    tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
                }
            }
            else
            {
                if(ball.self.norm()<2.0)
                {
                    static bool my_fisrt_lookat = true;
                    double dir = azimuth_deg(ball.self);
                    double dis = ball.self.norm();

                    if(dis>0.3)
                    {
                        my_fisrt_lookat = true;
                        tasks.push_back(make_shared<look_task>(HEAD_STATE_TRACK_BALL));
                        if(fabs(dir)>15.0)
                            tasks.push_back(make_shared<walk_task>(0.0, 0.0, sign(dir)*6.0, true));
                        else
                            tasks.push_back(make_shared<walk_task>(0.03, 0.0, 0.0, true));
                    }
                    else
                    {
                        double self2left_dir = azimuth_deg(WM->opp_post_left-self.global);
                        double self2right_dir = azimuth_deg(WM->opp_post_right-self.global);
                        if(self.dir>90.0)
                        {
                            tasks.push_back(make_shared<walk_task>(0.0, 0.01, -6.0, true));
                        }
                        else if(self.dir<-90.0)
                        {
                            tasks.push_back(make_shared<walk_task>(0.0, -0.01, 6.0, true));
                        }
                        else
                        {
                            tasks.push_back(make_shared<look_task>(0.0, 60.0));
                            if(my_fisrt_lookat)
                            {
                                my_fisrt_lookat = false;
                                return tasks;
                            }
                            if(ball.alpha>-0.1)
                            {
                                tasks.push_back(make_shared<walk_task>(0.0, -0.01, 0.0, true));
                            }
                            else if(ball.alpha<-0.22)
                            {
                                if(ball.beta>0.45)
                                    tasks.push_back(make_shared<walk_task>(-0.012, 0.0, 0.0, true));
                                else
                                    tasks.push_back(make_shared<walk_task>(0.0, 0.01, 0.0, true));
                            }
                            else
                            {
                                if(ball.beta<0.4)
                                    tasks.push_back(make_shared<walk_task>(0.012, 0.0, 0.0, true));
                                else if(ball.beta>0.45)
                                    tasks.push_back(make_shared<walk_task>(-0.01, 0.0, 0.0, true));
                                else
                                {
                                    tasks.push_back(make_shared<action_task>("left_little_kick"));
                                    keeper_kicked_ = true;
                                }
                            }
                        }
                    }
                }
            }
        }
        else
        {
            ball_block ball = WM->ball();
            self_block self = WM->self();
            if(see_last_&&!ball.can_see)
                SE->search_ball_end_ = false;
            if(ball.can_see)
            {
                tasks = play_skill_kick(self, ball);
            }
            else
            {
                tasks = play_skill_search_ball();
            }
            see_last_ = ball.can_see;
        }
    }
    return tasks;
}