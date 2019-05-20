#ifndef __FSM_STATE_KICK_BALL_HPP
#define __FSM_STATE_KICK_BALL_HPP

#include "fsm.hpp"
#include "engine/scan/scan_engine.hpp"

class FSMStateKickBall: public FSMState
{
public:
    FSMStateKickBall(FSM_Ptr fsm): FSMState(fsm)
    {
        exit_kick_dis_ = 0.45;
        retreat_alpha_ = 0.3;
        retreat_beta_ = 0.45;
    }
    
    task_list OnStateEnter()
    {
        LOG(LOG_INFO)<<"Enter kick ball"<<endll;
        task_list tasks;
        tasks.push_back(std::make_shared<LookTask>(0.0, motion::SE->pitch_range_[1]));
        return tasks;
    }

    task_list OnStateExit()
    {
        LOG(LOG_INFO)<<"Exit kick ball"<<endll;
        task_list tasks;
        return tasks;
    }

    task_list OnStateTick();

private:
    float exit_kick_dis_;
    float retreat_alpha_;
    float retreat_beta_;
};

#endif
