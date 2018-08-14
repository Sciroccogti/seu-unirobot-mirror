#include <iostream>
#include <csignal>
#include "configuration.hpp"
#include "robot/humanoid.hpp"
#include "options/options.hpp"
#include "tcp_server/tcp_server_handler.hpp"
#include "player/player.hpp"

using namespace std;
using namespace robot;
using namespace comm;

shared_ptr<player> maxwell;

void exit_handler(int sig)
{
    if(sig == SIGINT)
    {
        if(OPTS.use_debug()) TCP_SERVER.close();
        maxwell->kill();
    }
    cout<<"\n\033[32m*************************************************\033[0m\n";
    cout<<"\n\033[32m****************    Good bye!    ****************\033[0m\n";
    cout<<"\n\033[32m*************************************************\033[0m\n";
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
    if(OPTS.use_debug()) TCP_SERVER.start();
    signal(SIGINT, exit_handler);
    
    maxwell = make_shared<player>();
    maxwell->initialization();
    while(maxwell->is_alive())
    {
        sleep(2);
    }
    return 0;
}