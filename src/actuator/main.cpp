#include <iostream>
#include "configuration/configuration.hpp"
#include "robot/humanoid.hpp"

using namespace std;
using namespace config;
using namespace robot;

int main(int argc, char *argv[])
{
    if(!CONF.init(argc, argv))
    {
        cout<<"config init failed"<<endl;
        return 1;
    }
    ROBOT.init(CONF.robot_file(), CONF.action_file(), CONF.offset_file());
    return 0;
}