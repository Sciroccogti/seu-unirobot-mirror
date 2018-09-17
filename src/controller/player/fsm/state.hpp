#pragma once

#include <list>
#include <memory>
#include "plan/plan.hpp"

class player;
typedef std::shared_ptr<player> player_ptr;

namespace FSM
{
    class fsm;
    typedef std::shared_ptr<fsm> fsm_ptr;

    class state
    {
    public:
        state(const std::string &name , fsm_ptr _fsm)
        {
            name_ = name;
            fsm_ = _fsm;
        }
        virtual std::list<plan_ptr> run(player_ptr p) = 0;

        std::string name() const
        {
            return name_;
        }
    protected:
        fsm_ptr fsm_;
        std::string name_;
    };

    typedef std::shared_ptr<state> state_ptr;
}
