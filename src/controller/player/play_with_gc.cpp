#include <list>
#include "player.hpp"
#include "task/look_task.hpp"
#include "task/walk_task.hpp"
#include "task/action_task.hpp"
#include "core/worldmodel.hpp"
#include "configuration.hpp"
#include "math/math.hpp"
#include "vision/vision.hpp"
#include "skill/skill.hpp"

using namespace std;
using namespace robot;
using namespace Eigen;
using namespace robot_math;
using namespace motion;

std::list<task_ptr> player::play_with_gc()
{
    list<task_ptr> tasks;
    RoboCupGameControlData gc_data = WM->gc_data();
    Vector2f head_init = SE->head_init_deg_;
    int secondT = (int)gc_data.secondaryTime;
    int kickoff = (int)gc_data.kickOffTeam;
    int team_index = gc_data.teams[0].teamNumber == CONF->team_number()?0:1;

    switch (gc_data.state)
    {
        case STATE_INITIAL:
            if(kickoff == CONF->team_number() || kickoff == DROPBALL)
            {
                if(CONF->get_my_role()=="front")
                {
                    if(kickoff == CONF->team_number())
                        WM->set_my_pos(kickoff_pos_);
                    else
                        WM->set_my_pos(init_pos_);
                }
                else if(CONF->get_my_role()=="guard")
                {
                    if(WM->self().dir>45.0) 
                        WM->set_my_pos(Vector2d(start_pos_.x(), -start_pos_.y()));
                    else
                        WM->set_my_pos(start_pos_);
                }
                else if(CONF->get_my_role()=="keeper")
                {
                    WM->set_my_pos(init_pos_);
                }
            }
            else
            {
                if(WM->self().dir>45.0) 
                    WM->set_my_pos(Vector2d(start_pos_.x(), -start_pos_.y()));
                else
                    WM->set_my_pos(start_pos_);
            }
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1]));
            break;
        case STATE_READY:
            if(kickoff == CONF->team_number() || kickoff == DROPBALL)
            {
                if(CONF->get_my_role()=="front")
                {
                    if(!played_)
                    {
                       if(kickoff == CONF->team_number())
                            WM->set_my_pos(kickoff_pos_);
                        else
                            WM->set_my_pos(init_pos_);
                    }
                    else
                        tasks.push_back(skill_goto(WM->self(), init_pos_, 0.0));
                }
                else if(CONF->get_my_role()=="guard")
                {
                    tasks.push_back(skill_goto(WM->self(), init_pos_, 0.0));
                }
                else if(CONF->get_my_role()=="keeper")
                {
                    WM->set_my_pos(init_pos_);
                }
            }
            else
            {
                if(CONF->get_my_role()=="front")
                {
                    tasks.push_back(skill_goto(WM->self(), init_pos_, 0.0));
                }
                else if(CONF->get_my_role()=="guard")
                {
                    tasks.push_back(skill_goto(WM->self(), init_pos_, 0.0));
                }
                else if(CONF->get_my_role()=="keeper")
                {
                    WM->set_my_pos(init_pos_);
                }
            }
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1]));
            break;
        case STATE_SET:
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1]));
            break;
        case STATE_PLAYING:
            played_ = true;
            self_location_count_++;

            if(gc_data.teams[team_index].players[CONF->id()-1].penalty == HL_PICKUP_OR_INCAPABLE
                || gc_data.teams[team_index].players[CONF->id()-1].penalty == HL_SERVICE)
            {
                tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
                tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
                if(WM->self().dir>45.0) 
                    WM->set_my_pos(Vector2d(start_pos_.x(), -start_pos_.y()));
                else if(WM->self().dir<-45.0)
                    WM->set_my_pos(start_pos_);
            }
            else
            {
                if(kickoff != DROPBALL && kickoff!=CONF->team_number() && secondT!=0)
                    tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
                else
                {
                    task_list tlist = fsm_->Tick();
                    tasks.insert(tasks.end(), tlist.begin(), tlist.end());
                }
            }
            break;
        case STATE_FINISHED:
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1]));
            break;
        default:
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1]));
            break;
    }
    return tasks;
}