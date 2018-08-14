#include <iostream>
#include <csignal>
#include "configuration.hpp"
#include "robot/humanoid.hpp"
#include "options/options.hpp"
#include "player/player.hpp"

using namespace std;
using namespace robot;
using namespace comm;

shared_ptr<player> maxwell;

void exit_handler(int sig)
{
    cout<<"\n\033[32m*************************************************\033[0m\n";
    cout<<"\n\033[32m****************    Good bye!    ****************\033[0m\n";
    cout<<"\n\033[32m*************************************************\033[0m\n";
    if(sig == SIGINT)
    {
        maxwell->kill();
    }
}

int main(int argc, char *argv[])
{
    if(!OPTS.init(argc, argv))
    {
        cout<<"options init failed"<<endl;
        exit(1);
    }
    if(!CONF.init(OPTS.id()))
    {
        cout<<"config init failed"<<endl;
        exit(2);
    }
    ROBOT.init(CONF.robot_file(), CONF.action_file(), CONF.offset_file());
    signal(SIGINT, exit_handler);
    
    maxwell = make_shared<player>();
    maxwell->initialization();
    while(maxwell->is_alive())
    {
        sleep(2);
    }
    return 0;
}