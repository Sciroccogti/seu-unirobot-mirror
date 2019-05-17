#pragma once

#include "fsm.hpp"

class FSMStateGetup: public FSMState
{
public:
    FSMStateGetup(FSM_Ptr fsm): FSMState(fsm){}

    task_list OnStateEnter()
    {
        LOG(LOG_INFO)<<"Enter getup"<<endll;
        task_list tasks;
        return tasks;
    }

    task_list OnStateExit()
    {
        LOG(LOG_INFO)<<"Exit getup"<<endll;
        task_list tasks;
        return tasks;
    }

    task_list OnStateTick();
};
