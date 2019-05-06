#pragma once

#include "fsm.hpp"


class FSMStateGotoBall: public FSMState
{
public:
    FSMStateGotoBall(FSM_Ptr fsm): FSMState(fsm)
    {
        
    }
    
    task_list OnStateEnter()
    {
        task_list tasks;
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

        if(ball.self.norm()<0.2)
        {
            return fsm_->Trans(FSM_STATE_KICK_BALL);
        }
        
        return tasks;
    }
};
