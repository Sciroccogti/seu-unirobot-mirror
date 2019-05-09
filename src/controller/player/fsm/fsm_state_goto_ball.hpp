#pragma once

#include "fsm.hpp"


class FSMStateGotoBall: public FSMState
{
public:
    FSMStateGotoBall(FSM_Ptr fsm): FSMState(fsm)
    {
        enter_kick_dis_ = 0.25;
        retreat_x_dis_ = 0.15;
        retreat_y_dis_ = 0.15;
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

    task_list OnStateTick();

private:
    float enter_kick_dis_;
    float retreat_x_dis_;
    float retreat_y_dis_;
};
