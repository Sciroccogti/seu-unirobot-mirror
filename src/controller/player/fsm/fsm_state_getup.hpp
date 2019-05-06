#pragma once

#include "fsm.hpp"

class FSMStateGetup: public FSMState
{
public:
    FSMStateGetup(FSM_Ptr fsm): FSMState(fsm){}

    virtual task_list OnStateTick()
    {
        task_list tasks;
        if(WM->fall_data()!=FALL_NONE)
        {
            if(WM->fall_data()==FALL_FORWARD)
                tasks.push_back(std::make_shared<action_task>("front_getup"));
            else if(WM->fall_data()==FALL_BACKWARD)
                tasks.push_back(std::make_shared<action_task>("back_getup"));
            else if(WM->fall_data()==FALL_LEFT)
                tasks.push_back(std::make_shared<action_task>("right_arm"));
            else
                tasks.push_back(std::make_shared<action_task>("left_arm"));
        }
        else
        {
            fsm_->Trans(FSM_STATE_READY);
        }
        return tasks;
    }
};
