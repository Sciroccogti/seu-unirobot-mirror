#include "player.hpp"
#include "configuration.hpp"

using namespace std;

player::player(): agent(CONF.get_config_value<int>("period"))
{
}

bool player::initialization()
{
    if(!init()) return false;
    start_timer();
    return true;
}
