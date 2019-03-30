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

std::list<task_ptr> player::play_with_gc()
{
    list<task_ptr> tasks;
    RoboCupGameControlData gc_data = WM->gc_data();
    vector<float> head_init = CONF->get_config_vector<float>("scan.init");
    switch (gc_data.state)
    {
        case STATE_INITIAL:
            tasks.push_back(make_shared<action_task>("ready"));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], false));
            break;
        case STATE_READY:
            tasks.push_back(make_shared<action_task>("ready"));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], false));
            break;
        case STATE_SET:
            tasks.push_back(make_shared<action_task>("ready"));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], false));
            if(OPTS->use_robot())
                dynamic_pointer_cast<imu>(get_sensor("imu"))->set_zero();
            break;
        case STATE_PLAYING:
            if(WM->fall_data()!=FALL_NONE)
            {
                state_ = STATE_GETUP;
                tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], false));
                if(WM->fall_data()==FALL_FORWARD)
                    tasks.push_back(make_shared<action_task>("front_getup"));
                else if(WM->fall_data()==FALL_FORWARD)
                    tasks.push_back(make_shared<action_task>("back_getup"));
            }
            else
            {
                int secondT = (int)gc_data.secondaryTime;
                int kickoff = (int)gc_data.kickOffTeam;
                if(kickoff != DROPBALL && kickoff!=CONF->team_number() && secondT!=0)
                    tasks.push_back(make_shared<look_task>(true));
                else
                {
                    ball_block bb = WM->ball();
                    double d=bb.self.norm();
                    if(d<1.0)
                    {
                        state_=STATE_KICK;
                        Vector2d v=bb.pixel;
                        std::vector<double> head_degs = ROBOT->get_head_degs();
                        tasks.push_back(make_shared<look_task>(head_degs[0]-v[0], head_degs[1]+v[1], false));
                        
                    }
                    else
                    {
                        state_=STATE_SEARCH;
                        tasks.push_back(make_shared<look_task>(true));
                    } 
                }
            }
            break;
        case STATE_FINISHED:
            tasks.push_back(make_shared<action_task>("ready"));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], false));
            break;
        default:
            tasks.push_back(make_shared<action_task>("ready"));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], false));
            break;
    }
    return tasks;
}