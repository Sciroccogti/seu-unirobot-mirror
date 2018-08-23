#include <csignal>
#include <cstdlib>
#include <iomanip>
#include "configuration.hpp"
#include "robot/humanoid.hpp"
#include "options/options.hpp"
#include "player/player.hpp"
#include "logger.hpp"

using namespace std;
using namespace robot;

shared_ptr<player> maxwell;

void exit_handler(int sig)
{
    LOG(LOG_INFO)<<"\n--------------------------------------------------------\n";
    LOG(LOG_INFO)<<"                         Good bye!                      \n";
    LOG(LOG_INFO)<<"--------------------------------------------------------\n";
    if(sig == SIGINT)
    {
        maxwell->kill();
    }
}

void greeting()
{
    LOG(LOG_INFO)<<"\n--------------------------------------------------------\n";
    LOG(LOG_INFO)<<setw(20)<<"team-name: "<<CONF.get_config_value<string>("team_name")<<"\n";
    LOG(LOG_INFO)<<setw(20)<<"team-number: "<<CONF.get_config_value<string>("team_number")<<"\n";
    LOG(LOG_INFO)<<setw(20)<<"player-id: "<<CONF.id()<<"\n";
    LOG(LOG_INFO)<<  "--------------------------------------------------------\n";
}

int main(int argc, char *argv[])
{
    LOG.set_level(LOG_DEBUG);
    if(!OPTS.init(argc, argv))
    {
        LOG(LOG_ERROR)<<"options init failed\n";
        return 1;
    }
    if(!CONF.init(OPTS.id()))
    {
        LOG(LOG_ERROR)<<"config init failed\n";
        return 2;
    }
    ROBOT.init(CONF.robot_file(), CONF.action_file(), CONF.offset_file());
    signal(SIGINT, exit_handler);
    greeting();
    maxwell = make_shared<player>();
    if(!maxwell->initialization())
    {
        LOG(LOG_ERROR)<<"robot init failed\n";
        return 3;
    }

    while(maxwell->is_alive())
    {
        sleep(2);
    }
    return 0;
}