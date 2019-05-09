#pragma once

#include "fsm.hpp"
#include "vision/vision.hpp"

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
        VISION->localization_ = true;
        return tasks;
    }

    task_list OnStateExit()
    {
        task_list tasks;
        VISION->localization_ = false;
        return tasks;
    }

    task_list OnStateTick();
};

