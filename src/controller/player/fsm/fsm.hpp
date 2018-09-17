#pragma once

#include <memory>
#include <map>
#include "state.hpp"

namespace FSM
{
    typedef std::map<std::string, state_ptr> state_map;

    class fsm: public std::enable_shared_from_this<fsm>
    {
    public:
        fsm();
        void init(const std::string &name);
        bool set_state(const std::string &name);
        std::list<plan_ptr> run(player_ptr p);

        void load_state(state_ptr s)
        {
            state_map_[s->name()] = s;
        }

        state_ptr current_state()
        {
            return curr_state_;
        }
    private:
        state_map state_map_;
        state_ptr curr_state_;
    };
}
