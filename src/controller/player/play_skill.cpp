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
    self_block self = WM->self();
    Vector2d target_in_self = target-self.global;
    double dis = target_in_self.norm();
    
    if(dis>skill_goto_stop_distance)
    {
        //LOG(LOG_INFO)<<"far"<<endll;
        double azi_deg = azimuth_deg(target_in_self);
        double temp = normalize_deg(azi_deg-self.dir);
        if(fabs(temp)>skill_goto_turn_direction)
            return make_shared<walk_task>(0.0, 0.0, sign(temp)*8.0, true);
        else
            return make_shared<walk_task>(0.035, 0.0, 0.0, true);
    }
    else if(fabs(self.dir-dir)>skill_goto_stop_direction)
    {
        //LOG(LOG_INFO)<<"near"<<endll;
        double temp_dir = normalize_deg(dir-self.dir);
        if(fabs(temp_dir)>15.0)
            return make_shared<walk_task>(0.0, 0.0, sign(temp_dir)*8.0, true);
        else
            return make_shared<walk_task>(0.0, 0.0, sign(temp_dir)*6.0, true);
    }
    else
    {
        //LOG(LOG_INFO)<<"stop"<<endll;
        if(CONF->get_my_role()=="keeper")
            keeper_kicked_ = false;
        return make_shared<walk_task>(0.0, 0.0, 0.0, false);
    }
}

list<task_ptr> player::play_skill_kick(const self_block &self, const ball_block &ball)
{
    in_search_ball_ = false;
    static bool fisrt_lookat = true;
    static bool first_location = true;
    list<task_ptr> tasks;

    double dir = azimuth_deg(ball.self);
    double dis = ball.self.norm();

    if(dis>0.3)
    {
        fisrt_lookat = true;
        tasks.push_back(make_shared<look_task>(HEAD_STATE_TRACK_BALL));
        if(fabs(dir)>15.0)
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, sign(dir)*6.0, true));
        else
            tasks.push_back(make_shared<walk_task>(0.03, 0.0, 0.0, true));
    }
    else
    {
        if(self.global.x()<4.0)
        {
            double self2left_dir = azimuth_deg(WM->opp_post_left-self.global);
            double self2right_dir = azimuth_deg(WM->opp_post_right-self.global);
            if(self.dir>self2left_dir)
            {
                tasks.push_back(make_shared<walk_task>(-0.005, 0.01, -6.0, true));
            }
            else if(self.dir<self2right_dir)
            {
                tasks.push_back(make_shared<walk_task>(-0.005, -0.01, 6.0, true));
            }
            else
            {
                tasks.push_back(make_shared<look_task>(0.0, 60.0));
                if(fisrt_lookat)
                {
                    fisrt_lookat = false;
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
                        //if(!WM->kickoff_)
                            tasks.push_back(make_shared<action_task>("left_little_kick"));
                            /*
                        else
                        {
                            tasks.push_back(make_shared<action_task>("left_side_kick"));
                            WM->kickoff_ = false;
                        }
                        */
                        last_search_dir_ = WM->self().dir;
                    }
                }
            }
        }
        else
        {
            if(fabs(self.dir)>15.0)
            {
                tasks.push_back(make_shared<walk_task>(-0.01, 0.0, -sign(self.dir)*6.0, true));
            }
            else
            {
                tasks.push_back(make_shared<look_task>(0.0, 60.0));
                if(fisrt_lookat)
                {
                    fisrt_lookat = false;
                    return tasks;
                }
                if(ball.alpha>-0.1)
                {
                    tasks.push_back(make_shared<walk_task>(0.0, -0.01, 0.0, true));
                }
                else if(ball.alpha<-0.22)
                {
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
                        if(!WM->kickoff_)
                            tasks.push_back(make_shared<action_task>("left_little_kick"));
                        else
                        {
                            tasks.push_back(make_shared<action_task>("left_side_kick"));
                            WM->kickoff_ = false;
                        }
                        last_search_dir_ = WM->self().dir;
                    }
                }
            }
        }
    }
    return tasks;
}

list<task_ptr> player::play_skill_search_ball()
{
    in_search_ball_ = true;
    list<task_ptr> tasks;
    map< int, player_info > pinfos = WM->player_infos();
    
    /*
    if(pinfos.find(CONF->get_config_value<int>("strategy.front.id"))!=pinfos.end())
    {
        player_info keeper_info = pinfos[CONF->keeper_id()];
        if(keeper_info.can_see)
        {
            tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
            tasks.push_back(play_skill_goto(Vector2d(keeper_info.ball_x, keeper_info.ball_y), 0.0));
            return tasks;
        }
    }
    */
    if(SE->search_ball_end_)
    {
        double target_dir = normalize_deg(last_search_dir_+90.0);
        if(fabs(normalize_deg(target_dir-WM->self().dir))>skill_goto_turn_direction)
        {
            tasks.push_back(make_shared<look_task>(0.0, 45.0));
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, sign(normalize_deg(target_dir-WM->self().dir))*6.0, true));
        }
        else
        {
            last_search_dir_ = WM->self().dir;
            SE->search_ball_end_ = false;
            tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
            tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
        }
    }
    else
    {
        tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
        tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
    }
    return tasks;
}

task_ptr player::play_skill_penalty_kick(bool left, float init_dir)
{
    ball_block ball = WM->ball();
    self_block self = WM->self();
    if(left&&fabs(self.dir-init_dir)<10.0)
        return make_shared<walk_task>(0.0, -0.005, 5.0, true);
    if(!left&&fabs(self.dir-init_dir)<10.0)
        return make_shared<walk_task>(0.0, 0.005, -5.0, true);
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
            return make_shared<action_task>("penalty_kick");
    }
}

list<task_ptr> player::play_skill_localization()
{
    task_list tasks;
    vector<double> head_degs = ROBOT->get_head_degs();
    SE->lost_yaw_ = head_degs[0];
    SE->lost_pitch_ = head_degs[1];
    WM->in_localization_ = true;
    tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_POST));
    tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, false));
    return tasks;
}