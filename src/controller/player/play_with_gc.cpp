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
    
    switch (gc_data.state)
    {
        case STATE_INITIAL:
            if(WM->self().dir>45.0) 
                WM->set_my_pos(Vector2d(0.0, -3.0));
            else if(WM->self().dir<-45.0)
                WM->set_my_pos(Vector2d(0.0, 3.0));
            tasks.push_back(make_shared<action_task>("ready"));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            break;
        case STATE_READY:
            //tasks.push_back(play_skill_goto(Vector2d(-1.0, 0.0), 0.0));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            break;
        case STATE_SET:
            WM->reset_my_pos();
            //tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            //if(OPTS->use_robot())
            //    dynamic_pointer_cast<imu>(get_sensor("imu"))->set_zero();
            break;
        case STATE_PLAYING:
            if(WM->fall_data()!=FALL_NONE)
            {
                tasks.push_back(make_shared<look_task>(0.0, 0.0, HEAD_STATE_LOOKAT));
                if(WM->fall_data()==FALL_FORWARD)
                    tasks.push_back(make_shared<action_task>("front_getup"));
            }
            else
            {
                int team_index = gc_data.teams[0].teamNumber == CONF->team_number()?0:1;
                if(gc_data.teams[team_index].players[CONF->id()-1].penalty == HL_PICKUP_OR_INCAPABLE
                    || gc_data.teams[team_index].players[CONF->id()-1].penalty == HL_SERVICE)
                {
                    tasks.push_back(make_shared<look_task>(0.0, 0.0, HEAD_STATE_LOOKAT));
                    tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
                    if(WM->self().dir>45.0) 
                        WM->set_my_pos(Vector2d(0.0, -3.0));
                    else if(WM->self().dir<-45.0)
                        WM->set_my_pos(Vector2d(0.0, 3.0));
                }
                else
                {
                    int secondT = (int)gc_data.secondaryTime;
                    int kickoff = (int)gc_data.kickOffTeam;
                    if(kickoff != DROPBALL && kickoff!=CONF->team_number() && secondT!=0)
                        tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
                    else
                    {
                        if(role_ == "front")
                        {
                            ball_block ball = WM->ball();
                            self_block self = WM->self();
                            if(ball.can_see)
                            {
                                if(in_my_attack_range(ball.global))
                                {
                                    tasks = play_skill_front_kick(self, ball);
                                    WM->set_my_kick(true);
                                }
                                else
                                {
                                    tasks.push_back(play_skill_goto(Vector2d(0.0, 0.0), 0.0));
                                    WM->set_my_kick(false);
                                }
                            }
                            else
                                tasks = play_skill_search_ball();
                        }
                        else if(role_ == "guard")
                        {
                            ball_block ball = WM->ball();
                            self_block self = WM->self();
                            if(ball.can_see)
                            {
                                map< int, player_info > pinfos = WM->player_infos();
                                player_info g_info;
                                if(pinfos.find(CONF->get_config_value<int>("strategy.front.id"))!=pinfos.end())
                                {
                                    g_info = pinfos[CONF->get_config_value<int>("strategy.front.id")];
                                    if(in_my_attack_range(ball.global) && !g_info.my_kick)
                                    {
                                        tasks = play_skill_front_kick(self, ball);
                                        WM->set_my_kick(true);
                                    }
                                    else
                                    {
                                        tasks.push_back(play_skill_goto(Vector2d(-3.5, 0.0), 0.0));
                                        WM->set_my_kick(false);
                                    }
                                }
                                else
                                {
                                    tasks = play_skill_front_kick(self, ball);
                                    WM->set_my_kick(true);
                                }
                            }
                            else
                                tasks = play_skill_search_ball();
                        }
                    }
                }
            }
            break;
        case STATE_FINISHED:
            tasks.push_back(make_shared<action_task>("ready"));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            break;
        default:
            tasks.push_back(make_shared<action_task>("ready"));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            break;
    }
    return tasks;
}