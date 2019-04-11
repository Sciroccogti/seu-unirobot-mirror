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
            tasks.push_back(make_shared<action_task>("ready"));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            break;
        case STATE_READY:
            tasks.push_back(make_shared<action_task>("ready"));
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            break;
        case STATE_SET:
            tasks.push_back(make_shared<look_task>(head_init[0], head_init[1], HEAD_STATE_LOOKAT));
            //if(OPTS->use_robot())
            //    dynamic_pointer_cast<imu>(get_sensor("imu"))->set_zero();
            break;
        case STATE_PLAYING:
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