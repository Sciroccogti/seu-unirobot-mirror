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
const double skill_goto_stop_direction = 15.0;
const double skill_goto_turn_direction = 15.0;

task_ptr player::play_skill_goto(const Eigen::Vector2d &target, double dir)
{
    WM->self_localization_ = false;
    self_block self = WM->self();
    Vector2d target_in_self = target-self.global;
    double dis = target_in_self.norm();
    
    if(dis>skill_goto_stop_distance)
    {
        //LOG(LOG_INFO)<<"far"<<endll;
        double azi_deg = azimuth_deg(target_in_self);
        double temp = normalize_deg(azi_deg-self.dir);
        if(fabs(temp)>skill_goto_turn_direction)
            return make_shared<walk_task>(0.0, 0.0, sign(temp)*10.0, true);
        else
            return make_shared<walk_task>(0.04, 0.0, 0.0, true);
    }
    else if(fabs(self.dir-dir)>skill_goto_stop_direction)
    {
        //LOG(LOG_INFO)<<"near"<<endll;
        double temp_dir = normalize_deg(dir-self.dir);
        if(fabs(temp_dir)>15.0)
            return make_shared<walk_task>(0.0, 0.0, sign(temp_dir)*10.0, true);
        else
            return make_shared<walk_task>(0.0, 0.0, sign(temp_dir)*6.0, true);
    }
    else
    {
        //LOG(LOG_INFO)<<"stop"<<endll;
        return make_shared<walk_task>(0.0, 0.0, 0.0, false);
    }
}

list<task_ptr> player::play_skill_front_kick(const self_block &self, const ball_block &ball)
{
    WM->self_localization_ = false;
    static bool fisrt_lookat = true;
    list<task_ptr> tasks;

    double dir = azimuth_deg(ball.self);
    double dis = ball.self.norm();

    if(dis>0.3)
    {
        fisrt_lookat = true;
        tasks.push_back(make_shared<look_task>(HEAD_STATE_TRACK_BALL));
        if(fabs(dir)>15.0)
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, sign(dir)*8.0, true));
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
            tasks.push_back(make_shared<look_task>(0.0, 65.0, HEAD_STATE_LOOKAT));
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
                    tasks.push_back(make_shared<walk_task>(-0.02, 0.0, 0.0, true));
                else
                {
                    tasks.push_back(make_shared<action_task>("left_little_kick"));
                    SE->search_ball_circle_ = false;
                }
            }
        }
    }
    return tasks;
}

list<task_ptr> player::play_skill_search_ball()
{
    list<task_ptr> tasks;
    map< int, player_info > pinfos = WM->player_infos();
    static double last_dir = WM->self().dir;
    if(pinfos.find(CONF->keeper_id())!=pinfos.end())
    {
        player_info keeper_info = pinfos[CONF->keeper_id()];
        if(keeper_info.can_see)
        {
            tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
            tasks.push_back(play_skill_goto(Vector2d(keeper_info.ball_x, keeper_info.ball_y), 0.0));
            return tasks;
        }
    }
    if(SE->search_ball_circle_)
    {
        double target_dir = normalize_deg(last_dir+90.0);
        if(fabs(WM->self().dir-target_dir)>skill_goto_turn_direction)
        {
            WM->self_localization_ = false;
            tasks.push_back(make_shared<look_task>(0.0, 45.0, HEAD_STATE_LOOKAT));
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, 6.0, true));
        }
        else
        {
            last_dir = WM->self().dir;
            tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
        }
    }
    else
    {
        //WM->self_localization_ = true;
        tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
        tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
    }
    return tasks;
}