#pragma once

#include "fsm.hpp"


class FSMStateGotoBall: public FSMState
{
public:
    FSMStateGotoBall(FSM_Ptr fsm): FSMState(fsm)
    {
        
    }
    
    virtual void OnStateEnter(){};
    virtual void OnStateExit(){};
    virtual task_list OnStateTick()
    {
        task_list tasks;
        if(WM->fall_data()!=FALL_NONE)
        {
            fsm_->Trans(FSM_STATE_GETUP);
            return tasks;
        }
        
        ball_block ball = WM->ball();
        if(!ball.can_see)
        {
            fsm_->Trans(FSM_STATE_SEARCH_BALL);
            return tasks;
        }
        
        return tasks;
    }
};
