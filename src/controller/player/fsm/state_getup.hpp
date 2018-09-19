#pragma once

#include "state.hpp"

namespace FSM
{
    class state_getup: public state
    {
    public:
        state_getup(fsm_ptr _fsm): state("getup", _fsm)
        {

        }

        std::list<plan_ptr> run(player_ptr p)
        {
            std::list< plan_ptr > plist;
            return plist;
        }
    };
}