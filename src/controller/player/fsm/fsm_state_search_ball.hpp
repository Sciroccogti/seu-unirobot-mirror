#pragma once

#include "fsm.hpp"
#include <vector>
#include <eigen3/Eigen/Dense>
#include "math/math.hpp"

class FSMStateSearchBall: public FSMState
{
public:
    FSMStateSearchBall(FSM_Ptr fsm): FSMState(fsm)
    {
        last_search_dir_ = 0.0;
        head_pitch_min_angle_ = 0.0;
        head_pitch_mid_angle_ = 30.0;
        head_pitch_max_angle_ = 60.0;
        
        ball_search_table_.push_back(Eigen::Vector2f(head_pitch_min_angle_, 100.0));
        ball_search_table_.push_back(Eigen::Vector2f(head_pitch_min_angle_, 50.0));
        ball_search_table_.push_back(Eigen::Vector2f(head_pitch_min_angle_, 0.0));
        ball_search_table_.push_back(Eigen::Vector2f(head_pitch_min_angle_, -50.0));
        ball_search_table_.push_back(Eigen::Vector2f(head_pitch_min_angle_, -100.0));
        
        ball_search_table_.push_back(Eigen::Vector2f(head_pitch_mid_angle_, -85.0));
        ball_search_table_.push_back(Eigen::Vector2f(head_pitch_mid_angle_, -27.5));
        ball_search_table_.push_back(Eigen::Vector2f(head_pitch_mid_angle_, 27.5));
        ball_search_table_.push_back(Eigen::Vector2f(head_pitch_mid_angle_, 85.0));
        
        ball_search_table_.push_back(Eigen::Vector2f(head_pitch_max_angle_, 50.0));
        ball_search_table_.push_back(Eigen::Vector2f(head_pitch_max_angle_, 0.0));
        ball_search_table_.push_back(Eigen::Vector2f(head_pitch_max_angle_, -50.0));
    }
    
    virtual void OnStateEnter()
    {
        init_search_dir_ = WM->self().dir;
        last_search_dir_ = init_search_dir_;
        first_in_ = true;
    }
    
    virtual void OnStateExit()
    {
        //last_search_dir_
    }
    
    virtual task_list OnStateTick()
    {
        task_list tasks;
        if(WM->fall_data()!=FALL_NONE)
        {
            fsm_->Trans(FSM_STATE_GETUP);
            return tasks;
        }
        
        if(WM->ball().can_see)
        {
            fsm_->Trans(FSM_STATE_GOTO_BALL);
            return tasks;
        }
        
        if(first_in_)
        {
            first_in_ = false;
            tasks.push_back(std::make_shared<look_task>(ball_search_table_, motion::HEAD_STATE_SEARCH_BALL));
            tasks.push_back(std::make_shared<walk_task>(0.0, 0.0, 0.0, false));
        }
        else
        {
            if(motion::SE->search_ball_end_)
            {
                float my_dir = WM->self().dir;
                if(std::fabs(robot_math::normalize_deg(robot_math::normalize_deg(last_search_dir_+120.0)-my_dir))>10.0)
                {
                    tasks.push_back(std::make_shared<walk_task>(0.0, 0.0, 10.0, true));
                }
                else
                {
                    last_search_dir_ = my_dir;
                    tasks.push_back(std::make_shared<look_task>(ball_search_table_, motion::HEAD_STATE_SEARCH_BALL));
                    tasks.push_back(std::make_shared<walk_task>(0.0, 0.0, 0.0, false));
                }
            }
            else
            {
                tasks.push_back(std::make_shared<walk_task>(0.0, 0.0, 0.0, false));
            }
        }
        return tasks;
    }
    
private:
    float last_search_dir_;
    float init_search_dir_;
    
    bool first_in_;
    
    std::vector<Eigen::Vector2f> ball_search_table_;
    float head_pitch_min_angle_;
    float head_pitch_mid_angle_;
    float head_pitch_max_angle_;
};

