#pragma once

#include "fsm.hpp"

class FSMStateGetup: public FSMState
{
public:
    FSMStateGetup(FSM_Ptr fsm): FSMState(fsm){}

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
