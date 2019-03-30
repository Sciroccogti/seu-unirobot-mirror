#include "skill.hpp"
#include "task/action_task.hpp"
#include "task/look_task.hpp"
#include "task/walk_task.hpp"

using namespace std;
using namespace Eigen;
using namespace robot_math;

const double skill_goto_max_omnidirectional_distance = 0.5;
const double skill_goto_max_speed = 0.04;
const double skill_goto_stop_distance = 0.3;
const double skill_goto_stop_direction = 10.0;


task_ptr skill_goto_forward(const Vector2d &p, double dir);
task_ptr skill_goto_omnidirectional(const Vector2d &p, double dir);

task_ptr skill_goto(const Vector2d &p, double dir, bool omnidirectional)
{
    self_block me = WM->self();
    if(!omnidirectional&&(me.global-p).norm()>skill_goto_max_omnidirectional_distance)
    {
        return skill_goto_forward(p, dir);//make_shared<walk_task>(0,0,0,true);
    }
}

task_ptr kill_goto_forward(const Vector2d &p, double dir)
{
    self_block me = WM->self();
    double walkDir = azimuth((p - me.global));
    float drift_angle = normalize_deg(walkDir - me.dir);

    if (fabs(drift_angle) > 20.0f)
    {
        return make_shared<walk_task>(0,0,drift_angle,true);
    }
    else
    {
        return make_shared<walk_task>(skill_goto_max_speed,0,drift_angle,true);
    }
}

task_ptr skill_goto_omnidirectional(const Vector2d &p, double dir)
{
    self_block me=WM->self();
    double distance = (me.global-p).norm();
    double drift_angle = normalize_deg(dir-me.dir);
    if (distance < skill_goto_stop_distance && fabs(drift_angle) < skill_goto_stop_direction)
        return task_ptr();

    Vector2d relativeTarget = p-me.global;
    return make_shared<walk_task>(relativeTarget.x(), relativeTarget.y(), drift_angle, true);
}