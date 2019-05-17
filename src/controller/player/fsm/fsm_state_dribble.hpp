#pragma once

#include "fsm.hpp"
#include "engine/scan/ScanEngine.hpp"

class FSMStateDribble: public FSMState
{
public:
    FSMStateDribble(FSM_Ptr fsm): FSMState(fsm)
    {
        exit_kick_dis_ = 0.4;
        retreat_alpha_ = 0.25;
        retreat_beta_ = 0.3;
        target_pos_ = Eigen::Vector2d(5.0, 0.0);
    }
    
    task_list OnStateEnter()
    {
        LOG(LOG_INFO)<<"Enter dribble"<<endll;
        task_list tasks;
        tasks.push_back(std::make_shared<look_task>(0.0, motion::SE->pitch_range_[1]));
        return tasks;
    }

    task_list OnStateExit()
    {
        LOG(LOG_INFO)<<"Exit dribble"<<endll;
        task_list tasks;
        return tasks;
    }

    task_list OnStateTick();

private:
    float exit_kick_dis_;
    float retreat_alpha_;
    float retreat_beta_;

    Eigen::Vector2d target_pos_;
};
