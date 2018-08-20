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
    LOG(LOG_INFO, "\n--------------------------------------------------------");
    LOG(LOG_INFO,   "                         Good bye!                      ");
    LOG(LOG_INFO,   "--------------------------------------------------------");
    if(sig == SIGINT)
    {
        maxwell->kill();
    }
}

void greeting()
{
    LOG(LOG_INFO, "\n--------------------------------------------------------");
    LOG(LOG_INFO, setiosflags(ios::right)<<setw(20)<<"team-name: "<<CONF.get_config_value<string>("team_name"));
    LOG(LOG_INFO, setiosflags(ios::right)<<setw(20)<<"team-number: "<<CONF.get_config_value<string>("team_number"));
    LOG(LOG_INFO, setiosflags(ios::right)<<setw(20)<<"player-id: "<<CONF.id());
    LOG(LOG_INFO,   "--------------------------------------------------------");
}

int main(int argc, char *argv[])
{
    /*
    int fd_ = open("/dev/video0", O_RDWR,0);
    if(fd_<0)
    {
        LOG(LOG_ERROR, "open camera failed");
        return 1;
    }

    
    struct v4l2_capability vc;
    if(ioctl(fd_, VIDIOC_QUERYCAP, &vc) !=-1)
    {
        LOG(LOG_INFO, "driver:\t"<<vc.driver);
        LOG(LOG_INFO, "card:\t"<<vc.card);
        LOG(LOG_INFO, "bus_info:\t"<<vc.bus_info);
        LOG(LOG_INFO, "version:\t"<<vc.version);
    }
    
    v4l2_fmtdesc fmtdesc;
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    LOG(LOG_INFO, "Support format:");
    while (ioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc) != -1)
    {
        LOG(LOG_INFO, '\t'<<fmtdesc.index<< ". " << fmtdesc.description);
        fmtdesc.index++;
    }
    close(fd_);
    return 0;
    */
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
    greeting();
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