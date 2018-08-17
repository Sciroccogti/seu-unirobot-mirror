#include <QApplication>
#include "static_action.hpp"
#include "configuration.hpp"
#include "options/options.hpp"
#include "logger.hpp"

using namespace std;
using namespace robot;

int main(int argc, char **argv)
{
    if(!OPTS.init(argc, argv))
    {
        LOG(LOG_ERROR, "options init failed");
        exit(1);
    }
    if(!CONF.init(OPTS.id()))
    {
        LOG(LOG_ERROR, "config init failed");
        exit(2);
    }
    ROBOT.init(CONF.robot_file(), CONF.action_file(), CONF.offset_file());

    QApplication app(argc, argv);
    glutInit(&argc, argv);
    static_action foo;
    //foo.showMaximized();
    foo.show();
    return app.exec();
}
