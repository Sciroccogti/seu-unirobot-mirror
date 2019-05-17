#include <memory>
#include "skill.hpp"
#include "task/walk_task.hpp"
#include "task/action_task.hpp"
#include "task/look_task.hpp"

using namespace std;
using namespace Eigen;
using namespace robot_math;

const double skill_goto_max_speed = 0.035;
const double skill_goto_stop_distance = 0.2;
const double skill_goto_stop_direction = 10.0;
const double skill_goto_turn_direction = 10.0;

task_ptr skill_goto(const self_block &self, const Eigen::Vector2d &target, double dir)
{
    Vector2d target_in_self = target-self.global;
    double dis = target_in_self.norm();
    
    if(dis>skill_goto_stop_distance)
    {
        double azi_deg = azimuth_deg(target_in_self);
        double temp = normalize_deg(azi_deg-self.dir);
        bound(-skill_goto_turn_direction, skill_goto_turn_direction, temp);
        return make_shared<walk_task>(skill_goto_max_speed, 0.0, temp, true);
    }
    else if(fabs(self.dir-dir)>skill_goto_stop_direction)
    {
        double temp_dir = normalize_deg(dir-self.dir);
        bound(-skill_goto_turn_direction, skill_goto_turn_direction, temp_dir);
        return make_shared<walk_task>(0.0, 0.0, temp_dir, true);
    }
    else
    {
        return make_shared<walk_task>(0.0, 0.0, 0.0, false);
    }
}

task_ptr skill_penalty_kick(const ball_block &ball)
{
    if(ball.alpha>-0.05)
    {
        return make_shared<walk_task>(0.0, -0.01, 0.0, true);
    }
    else if(ball.alpha<-0.15)
    {
        return make_shared<walk_task>(0.0, 0.01, 0.0, true);
    }
    else
    {
        if(ball.beta<0.32)
            return make_shared<walk_task>(0.01, 0.0, 0.0, true);
        else if(ball.beta>0.4)
            return make_shared<walk_task>(-0.01, 0.0, 0.0, true);
        else
            return make_shared<action_task>("left_kick");
    }
}