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

    task_list OnStateTick();
};
