#ifndef SEU_UNIROBOT_PLAYER_HPP
#define SEU_UNIROBOT_PLAYER_HPP

#include "core/agent.hpp"

class player: public agent, public std::enable_shared_from_this<player>
{
public:
    player();
    bool initialization();
    std::shared_ptr<robot_subscriber> suber()
    {
        return suber_;
    }
protected:
    std::list<plan_ptr> think();

private:
    std::list<plan_ptr> play_with_remote();
};

#endif