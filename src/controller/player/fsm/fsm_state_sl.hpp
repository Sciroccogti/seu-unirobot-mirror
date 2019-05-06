#pragma once

#include "fsm.hpp"

class FSMStateSL: public FSMState
{
public:
    FSMStateSL(FSM_Ptr fsm): FSMState(fsm)
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
        return tasks;
    }
};

