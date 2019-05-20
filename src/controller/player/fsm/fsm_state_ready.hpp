#ifndef __FSM_STATE_READY_HPP
#define __FSM_STATE_READY_HPP

#include "fsm.hpp"

class FSMStateReady: public FSMState
{
public:
    FSMStateReady(FSM_Ptr fsm): FSMState(fsm){}

    task_list OnStateEnter()
    {
        LOG(LOG_INFO)<<"Enter ready"<<endll;
        task_list tasks;
        return tasks;
    }

    task_list OnStateExit()
    {
        LOG(LOG_INFO)<<"Exit ready"<<endll;
        task_list tasks;
        return tasks;
    }

    task_list OnStateTick();
};

#endif
