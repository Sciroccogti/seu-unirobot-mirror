#include "fsm.hpp"
#include "state_reset.hpp"
#include "state_getup.hpp"
#include "state_stance.hpp"

namespace FSM
{
    using namespace std;

    fsm::fsm()
    {
        state_map_.clear();
        curr_state_.reset();
    }

    void fsm::init(const std::string &name)
    {
        load_state(make_shared<state_getup>(shared_from_this()));
        load_state(make_shared<state_reset>(shared_from_this()));
        load_state(make_shared<state_stance>(shared_from_this()));
        set_state(name);
    }

    bool fsm::set_state(const std::string &name)
    {
        state_map::iterator iter;
        iter = state_map_.find(name);

        if (iter == state_map_.end())
        {
            return false;
        }

        curr_state_ = iter->second;
        return true;
    }

    std::list<plan_ptr> fsm::run(player_ptr p)
    {
        list<plan_ptr> plist;

        if (curr_state_.get())
        {
            plist = curr_state_->run(p);
        }

        return plist;
    }
}