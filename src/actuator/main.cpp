#include <iostream>
#include "configuration.hpp"
#include "robot/humanoid.hpp"
#include "options/options.hpp"

using namespace std;
using namespace robot;

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
    return 0;
}