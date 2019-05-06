#pragma once

#include "fsm.hpp"

class FSMStateReady: public FSMState
{
public:
    FSMStateReady(FSM_Ptr fsm): FSMState(fsm){}

    virtual task_list OnStateTick()
    {
        task_list tasks;
        if(WM->fall_data()!=FALL_NONE)
        {
            fsm_->Trans(FSM_STATE_GETUP);
            return tasks;
        }
        tasks.push_back(std::make_shared<walk_task>(0.0, 0.0, 0.0, false));
        tasks.push_back(std::make_shared<look_task>(0.0, 40.0));
        
        fsm_->Trans(FSM_STATE_SEARCH_BALL);
        return tasks;
    }
};
