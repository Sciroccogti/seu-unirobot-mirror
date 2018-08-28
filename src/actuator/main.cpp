#include <csignal>
#include <cstdlib>
#include <iomanip>
#include "configuration.hpp"
#include "robot/humanoid.hpp"
#include "options/options.hpp"
#include "player/player.hpp"


using namespace std;
using namespace robot;

shared_ptr<player> maxwell;

void exit_handler(int sig)
{
    std::cout<<"\n\033[32m--------------------------------------------------------\n";
    std::cout<<"                         Good bye!                      \n";
    std::cout<<"--------------------------------------------------------\n";
    if(sig == SIGINT)
    {
        maxwell->stop();
        sleep(1);
    }
}

void greeting()
{
    std::cout<<"\033[32m--------------------------------------------------------\n";
    std::cout<<left<<setw(15)<<"team-name: "<<CONF.get_config_value<string>("team_name")<<"\n";
    std::cout<<left<<setw(15)<<"team-number: "<<CONF.get_config_value<string>("team_number")<<"\n";
    std::cout<<left<<setw(15)<<"player-id: "<<CONF.id()<<"\n";
    std::cout<<  "--------------------------------------------------------\n\033[0m";
}

int main(int argc, char *argv[])
{
    if(!OPTS.init(argc, argv))
    {
        std::cout<<"\033[31moptions init failed\n\033[0m";
        return 1;
    }
    if(!CONF.init(OPTS.id()))
    {
        std::cout<<"\033[31mconfig init failed\n\033[0m";
        return 2;
    }
    ROBOT.init(CONF.robot_file(), CONF.action_file(), CONF.offset_file());
    signal(SIGINT, exit_handler);
    greeting();
    maxwell = make_shared<player>();
    if(!maxwell->initialization())
    {
        std::cout<<"\033[31mrobot init failed\n\033[0m";
        return 3;
    }
    while(maxwell->is_alive())
    {
        sleep(2);
    }
    return 0;
}