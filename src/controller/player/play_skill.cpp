#include "player.hpp"
#include "task/action_task.hpp"
#include "task/look_task.hpp"
#include "task/walk_task.hpp"

using namespace std;
using namespace Eigen;
using namespace robot_math;
using namespace robot;
using namespace motion;

const double skill_goto_max_speed = 0.03;
const double skill_goto_stop_distance = 0.2;
const double skill_goto_stop_direction = 10.0;
const double skill_goto_turn_direction = 10.0;

task_ptr player::play_skill_goto(const Eigen::Vector2d &target, double dir)
{
    self_block self = WM->self();
    double dis = (target-self.global).norm();
    Vector2d target_in_self = target-self.global;
    if(dis<=skill_goto_stop_distance && fabs(self.dir-dir)<=skill_goto_stop_direction)
        return make_shared<walk_task>(0.0, 0.0, 0.0, false);
    else if(dis>skill_goto_stop_distance)
    {
        double azi_deg = azimuth_deg(target_in_self);
        if(fabs(azi_deg)>skill_goto_turn_direction)
            return make_shared<walk_task>(0.0, 0.0, sign(azi_deg)*6.0, true);
        else
            return make_shared<walk_task>(skill_goto_max_speed, 0.0, 0.0, true);
    }
    else
    {
        double temp_dir = normalize_deg(dir-self.dir);
        return make_shared<walk_task>(0.0, 0.0, sign(temp_dir)*6.0, true);
    }
}

list<task_ptr> player::play_skill_front_kick(const self_block &self, const ball_block &ball)
{
    static bool fisrt_lookat = true;
    list<task_ptr> tasks;

    double dir = azimuth_deg(ball.self);
    double dis = ball.self.norm();

    if(dis>0.3)
    {
        fisrt_lookat = true;
        tasks.push_back(make_shared<look_task>(HEAD_STATE_TRACK_BALL));
        if(fabs(dir)>15.0)
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, 8.0, true));
        else
            tasks.push_back(make_shared<walk_task>(0.03, 0.0, 0.0, true));
    }
    else
    {
        double self2left_dir = azimuth_deg(WM->opp_post_left-self.global);
        double self2right_dir = azimuth_deg(WM->opp_post_right-self.global);
        if(self.dir>self2left_dir)
        {
            tasks.push_back(make_shared<walk_task>(0.0, 0.01, -6.0, true));
        }
        else if(self.dir<self2right_dir)
        {
            tasks.push_back(make_shared<walk_task>(0.0, -0.01, 6.0, true));
        }
        else
        {
            tasks.push_back(make_shared<look_task>(0.0, 70.0, HEAD_STATE_LOOKAT));
            if(fisrt_lookat)
            {
                fisrt_lookat = false;
                return tasks;
            }
            if(ball.alpha>-0.1)
            {
                tasks.push_back(make_shared<walk_task>(0.0, -0.01, 0.0, true));
            }
            else if(ball.alpha<-0.25)
            {
                tasks.push_back(make_shared<walk_task>(0.0, 0.01, 0.0, true));
            }
            else
            {
                if(ball.beta<0.3)
                    tasks.push_back(make_shared<walk_task>(0.015, 0.0, 0.0, true));
                else if(ball.beta>0.4)
                    tasks.push_back(make_shared<walk_task>(0.015, 0.0, 0.0, true));
                else
                {
                    tasks.push_back(make_shared<action_task>("left_little_kick"));
                }
            }
        }
    }
    return tasks;
}

list<task_ptr> player::play_skill_search_ball(const self_block &self)
{
    list<task_ptr> tasks;
    map< int, player_info > pinfos = WM->player_infos();
    if(pinfos.find(CONF->keeper_id())!=pinfos.end())
    {
        player_info keeper_info = pinfos[CONF->keeper_id()];
        if(keeper_info.can_see)
        {
            tasks.push_back(play_skill_goto(Vector2d(keeper_info.ball_x, keeper_info.ball_y), 0.0));
        }
    }
    return tasks;
}