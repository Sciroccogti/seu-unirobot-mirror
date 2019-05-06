#pragma once

#include "fsm.hpp"

class FSMStateReady: public FSMState
{
public:
    FSMStateReady(FSM_Ptr fsm): FSMState(fsm){}

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
        if(WM->fall_data()!=FALL_NONE)
        {
            return fsm_->Trans(FSM_STATE_GETUP);;
        }
        task_list tasks;
        tasks.push_back(std::make_shared<walk_task>(0.0, 0.0, 0.0, false));
        tasks.push_back(std::make_shared<look_task>(0.0, 40.0));
        return tasks;
    }
};
