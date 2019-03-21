#include <list>
#include "player.hpp"
#include "task/look_task.hpp"
#include "task/walk_task.hpp"
#include "task/action_task.hpp"
#include "core/worldmodel.hpp"
#include "configuration.hpp"

using namespace std;
using namespace robot;

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
            dynamic_pointer_cast<imu>(get_sensor("imu"))->set_zero();
            break;
        case STATE_PLAYING:
            if(WM->fall_data()!=FALL_NONE)
            {
                tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], false));
                if(WM->fall_data()==FALL_FORWARD)
                    tasks.push_back(make_shared<action_task>("getup_front"));
                else if(WM->fall_data()==FALL_FORWARD)
                    tasks.push_back(make_shared<action_task>("getup_back"));
            }
            else
            {
                
            }
            break;
        case STATE_FINISHED:
            tasks.push_back(make_shared<action_task>("ready"));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], false));
            break;
        default:
            break;
    }
    return tasks;
}