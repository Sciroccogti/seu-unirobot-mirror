#pragma once

#include "fsm.hpp"

class FSMStateKickBall: public FSMState
{
public:
    FSMStateKickBall(FSM_Ptr fsm): FSMState(fsm)
    {
        
    }
    
    task_list OnStateEnter()
    {
        task_list tasks;
        tasks.push_back(std::make_shared<look_task>(0.0, 60.0));
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
        
        ball_block ball = WM->ball();
        if(!ball.can_see)
        {
            return fsm_->Trans(FSM_STATE_SEARCH_BALL);
        }
        
        if(ball.alpha>-0.1)
        {
            tasks.push_back(std::make_shared<walk_task>(0.0, -0.01, 0.0, true));
        }
        else if(ball.alpha<-0.22)
        {
            tasks.push_back(std::make_shared<walk_task>(0.0, 0.01, 0.0, true));
        }
        else
        {
            if(ball.beta<0.4)
                tasks.push_back(std::make_shared<walk_task>(0.012, 0.0, 0.0, true));
            else if(ball.beta>0.45)
                tasks.push_back(std::make_shared<walk_task>(-0.01, 0.0, 0.0, true));
            else
            {
                tasks.push_back(std::make_shared<action_task>("penalty_kick"));
            }
        }

        return tasks;
    }
};
