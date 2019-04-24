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

std::list<task_ptr> player::play_with_gc()
{
    list<task_ptr> tasks;
    RoboCupGameControlData gc_data = WM->gc_data();
    vector<float> head_init = CONF->get_config_vector<float>("scan.init");
    int secondT = (int)gc_data.secondaryTime;
    int kickoff = (int)gc_data.kickOffTeam;
    int team_index = gc_data.teams[0].teamNumber == CONF->team_number()?0:1;
    //LOG(LOG_INFO)<<CONF->get_my_role()<<'\t'<<(int)gc_data.state<<endll;
    switch (gc_data.state)
    {
        case STATE_INITIAL:
            if(kickoff == CONF->team_number() || kickoff == DROPBALL)
            {
                if(CONF->get_my_role()=="front")
                {
                    WM->set_my_pos(Vector2d(-0.6, 0.0));
                }
                else if(CONF->get_my_role()=="guard")
                {
                    if(WM->self().dir>45.0) 
                    {
                        WM->set_my_pos(Vector2d(-0.75, -3.0));
                    }
                    else
                    {
                        WM->set_my_pos(Vector2d(-0.75, 3.0));
                    }
                }
                else if(CONF->get_my_role()=="keeper")
                {
                    WM->set_my_pos(Vector2d(-4.5, 0.0));
                }
            }
            else
            {
                if(WM->self().dir>45.0) 
                {
                    WM->set_my_pos(Vector2d(-0.75, -3.0));
                }
                else
                {
                    WM->set_my_pos(Vector2d(-0.75, 3.0));
                }
            }
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            break;
        case STATE_READY:
            if(kickoff == CONF->team_number() || kickoff == DROPBALL)
            {
                if(CONF->get_my_role()=="front")
                {
                    if(!played_)
                        WM->set_my_pos(Vector2d(-0.6, 0.0));
                    else
                        tasks.push_back(play_skill_goto(Vector2d(-1.25, 0.0), 0.0));
                }
                else if(CONF->get_my_role()=="guard")
                {
                    tasks.push_back(play_skill_goto(Vector2d(-2.5, 0.0), 0.0));
                }
                else if(CONF->get_my_role()=="keeper")
                {
                    WM->set_my_pos(Vector2d(-4.5, 0.0));
                }
            }
            else
            {
                if(CONF->get_my_role()=="front")
                {
                    tasks.push_back(play_skill_goto(Vector2d(-1.25, 0.0), 0.0));
                }
                else if(CONF->get_my_role()=="guard")
                {
                    tasks.push_back(play_skill_goto(Vector2d(-2.5, 0.0), 0.0));
                }
                else if(CONF->get_my_role()=="keeper")
                {
                    WM->set_my_pos(Vector2d(-4.5, 0.0));
                }
            }
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            break;
        case STATE_SET:
            WM->kickoff_ = true;
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            break;
        case STATE_PLAYING:
            played_ = true;
            self_location_count_++;
            if(WM->fall_data()!=FALL_NONE)
            {
                tasks.push_back(make_shared<look_task>(HEAD_STATE_LOOKAT));
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
                if(gc_data.teams[team_index].players[CONF->id()-1].penalty == HL_PICKUP_OR_INCAPABLE
                    || gc_data.teams[team_index].players[CONF->id()-1].penalty == HL_SERVICE)
                {
                    tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
                    tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
                    if(WM->self().dir>45.0) 
                        WM->set_my_pos(Vector2d(0.0, -3.0));
                    else if(WM->self().dir<-45.0)
                        WM->set_my_pos(Vector2d(0.0, 3.0));
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
                                        tasks.push_back(make_shared<look_task>(0.0, 60.0, HEAD_STATE_LOOKAT));
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
                        if(kickoff != DROPBALL && kickoff!=CONF->team_number() && secondT!=0)
                            tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
                        else
                        {
                            ball_block ball = WM->ball();
                            self_block self = WM->self();
                            map< int, player_info > pinfos = WM->player_infos();
                            player_info f_info;
                            int other_id = CONF->id()%2+1;
                            if(pinfos.find(other_id)!=pinfos.end())
                                f_info = pinfos[other_id];
                            if(see_last_&&!ball.can_see)
                                SE->search_ball_circle_ = false;
                            if(ball.can_see)
                            {
                                if(!f_info.my_kick&&!WM->my_info().my_kick)
                                {
                                    double other_dis = (Vector2d(f_info.ball_x, f_info.ball_y)-Vector2d(f_info.x, f_info.y)).norm();
                                    if(!f_info.can_see)
                                        other_dis = 10000.0;
                                    double my_dis = ball.self.norm();
                                    if(my_dis<other_dis)
                                    {
                                        WM->set_my_kick(true);
                                        tasks = play_skill_kick(self, ball);
                                    }
                                    else
                                    {
                                        WM->set_my_kick(false);
                                        tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
                                    }
                                }
                                else if(WM->my_info().my_kick)
                                {
                                    double other_dis = (Vector2d(f_info.ball_x, f_info.ball_y)-Vector2d(f_info.x, f_info.y)).norm();
                                    if(!f_info.can_see)
                                        other_dis = 10000.0;
                                    double my_dis = ball.self.norm();
                                    if(my_dis-other_dis<0.5)
                                    {
                                        WM->set_my_kick(true);
                                        tasks = play_skill_kick(self, ball);
                                    }
                                    else
                                    {
                                        WM->set_my_kick(false);
                                        tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
                                    }
                                }
                                else
                                {
                                    WM->set_my_kick(false);
                                    tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
                                }
                            }
                            else
                            {
                                WM->set_my_kick(false);
                                if(f_info.can_see && f_info.my_kick)
                                    tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
                                else
                                    tasks = play_skill_search_ball();
                            }
                            see_last_ = ball.can_see;
                        }
                    }
                }
            }
            break;
        case STATE_FINISHED:
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            break;
        default:
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            break;
    }
    return tasks;
}