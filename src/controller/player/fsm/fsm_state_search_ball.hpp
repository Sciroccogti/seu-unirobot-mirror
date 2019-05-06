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
    }
    
    task_list OnStateEnter()
    {
        task_list tasks;
        init_search_dir_ = WM->self().dir;
        last_search_dir_ = init_search_dir_;
        first_in_ = true;
        return tasks;
    }

    task_list OnStateExit()
    {
        task_list tasks;
        return tasks;
    }
    
    task_list OnStateTick()
    {
        task_list tasks;
        if(WM->fall_data()!=FALL_NONE)
        {
            return fsm_->Trans(FSM_STATE_GETUP);
        }
        
        if(WM->ball().can_see)
        {
            return fsm_->Trans(FSM_STATE_GOTO_BALL);
        }
        
        if(first_in_)
        {
            first_in_ = false;
            tasks.push_back(std::make_shared<look_task>(motion::HEAD_STATE_SEARCH_BALL));
            tasks.push_back(std::make_shared<walk_task>(0.0, 0.0, 0.0, false));
        }
        else
        {
            /*
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
            */
            tasks.push_back(std::make_shared<look_task>(motion::HEAD_STATE_SEARCH_BALL));
            tasks.push_back(std::make_shared<walk_task>(0.0, 0.0, 10.0, true));
        }
        return tasks;
    }
    
private:
    float last_search_dir_;
    float init_search_dir_;
    
    bool first_in_;
};

