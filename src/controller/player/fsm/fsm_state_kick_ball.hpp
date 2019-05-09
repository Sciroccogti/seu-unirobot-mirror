#pragma once

#include "fsm.hpp"

class FSMStateKickBall: public FSMState
{
public:
    FSMStateKickBall(FSM_Ptr fsm): FSMState(fsm)
    {
        exit_kick_dis_ = 0.4;
        retreat_alpha_ = 0.4;
        retreat_beta_ = 0.47;
    }
    
    task_list OnStateEnter()
    {
        task_list tasks;
        tasks.push_back(std::make_shared<look_task>(0.0, 60.0));
        return tasks;
    }

    task_list OnStateExit()
    {
        task_list tasks;
        return tasks;
    }

    task_list OnStateTick();

private:
    float exit_kick_dis_;
    float retreat_alpha_;
    float retreat_beta_;
};
