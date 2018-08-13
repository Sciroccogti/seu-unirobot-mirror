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
};

#endif