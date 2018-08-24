#ifndef SEU_UNIROBOT_ACTUATOR_STATE_RESET_HPP
#define SEU_UNIROBOT_ACTUATOR_STATE_RESET_HPP

#include "state.hpp"
#include "plan/walk_plan.hpp"

namespace FSM
{
    class state_reset: public state
    {
    public:
        state_reset(fsm_ptr _fsm): state("reset", _fsm)
        {

        }

        std::list<plan_ptr> run(player_ptr p)
        {
            std::list<plan_ptr> plist;
            fsm_->set_state("stance");
            return plist;
        }
    };
}

#endif
