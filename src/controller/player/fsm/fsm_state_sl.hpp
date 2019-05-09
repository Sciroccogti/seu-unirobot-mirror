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
        motion::SE->search_post_end_ = false;
        return tasks;
    }

    task_list OnStateExit()
    {
        task_list tasks;
        return tasks;
    }

    task_list OnStateTick();
};

