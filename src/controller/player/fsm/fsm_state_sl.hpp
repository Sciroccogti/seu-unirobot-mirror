#pragma once

#include "fsm.hpp"

class FSMStateSL: public FSMState
{
public:
    FSMStateSL(FSM_Ptr fsm): FSMState(fsm)
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
        return tasks;
    }
};

