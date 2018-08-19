#include <csignal>
#include <cstdlib>
#include "configuration.hpp"
#include "robot/humanoid.hpp"
#include "options/options.hpp"
#include "player/player.hpp"
#include "logger.hpp"

using namespace std;
using namespace robot;
using namespace comm;

shared_ptr<player> maxwell;

void exit_handler(int sig)
{
    LOG(LOG_INFO, "\n*************************************************");
    LOG(LOG_INFO, "****************    Good bye!    ****************");
    LOG(LOG_INFO, "*************************************************");
    if(sig == SIGINT)
    {
        maxwell->kill();
    }
}

int main(int argc, char *argv[])
{
    if(!OPTS.init(argc, argv))
    {
        LOG(LOG_ERROR, "options init failed");
        return 1;
    }
    if(!CONF.init(OPTS.id()))
    {
        LOG(LOG_ERROR, "config init failed");
        return 2;
    }
    ROBOT.init(CONF.robot_file(), CONF.action_file(), CONF.offset_file());
    signal(SIGINT, exit_handler);

    maxwell = make_shared<player>();
    if(!maxwell->initialization())
    {
        LOG(LOG_ERROR, "robot init failed");
        return 3;
    }

    while(maxwell->is_alive())
    {
        
        sleep(2);
    }
    return 0;
}