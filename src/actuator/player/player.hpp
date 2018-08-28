#ifndef SEU_UNIROBOT_PLAYER_HPP
#define SEU_UNIROBOT_PLAYER_HPP

#include <list>
#include "core/agent.hpp"
#include "core/world_model.hpp"
#include "fsm/fsm.hpp"
#include "walk/WalkEngine.hpp"

class player: public agent, public std::enable_shared_from_this<player>
{
public:
    player();
    bool initialization();
    std::shared_ptr<robot_subscriber> suber()
    {
        return suber_;
    }

    void stop()
    {
        kill();
        walk::WALK.stop();
    }

protected:
    std::list<plan_ptr> think();
private:
    std::list<plan_ptr> play_with_remote();
    std::list<plan_ptr> play_with_gamectrl();
    std::list<plan_ptr> play_without_gamectrl();
    FSM::fsm_ptr fsm_;
};

#endif