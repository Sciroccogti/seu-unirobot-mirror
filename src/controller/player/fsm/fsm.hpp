#pragma once

#include <memory>
#include <unordered_map>
#include "model.hpp"
#include "task/task.hpp"
#include "task/walk_task.hpp"
#include "task/action_task.hpp"
#include "task/look_task.hpp"
#include "core/worldmodel.hpp"
#include "engine/scan/ScanEngine.hpp"

class FSM;

typedef std::shared_ptr<FSM> FSM_Ptr;

class FSMState
{
public:
    FSMState(FSM_Ptr fsm): fsm_(fsm){}

    virtual void OnStateEnter(){};
    virtual void OnStateExit(){};
    virtual task_list OnStateTick() = 0;

protected:
    FSM_Ptr fsm_;
};

typedef std::shared_ptr<FSMState> FSMState_Ptr;

class FSM
{
public:
    bool Trans(fsm_state state)
    {
        states_[current_state_]->OnStateExit();
        states_[state]->OnStateEnter();
        current_state_ = state;
        return true;
    }

    task_list Tick()
    {
        return states_[current_state_]->OnStateTick();
    }
    
    bool Register(fsm_state s, FSMState_Ptr state)
    {
        states_[s] = state;
        return true;
    }

    fsm_state State()
    {
        return current_state_;
    }

private:
    std::unordered_map<int, FSMState_Ptr> states_;
    fsm_state current_state_;
};
