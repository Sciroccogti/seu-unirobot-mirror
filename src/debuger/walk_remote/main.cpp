#include <QApplication>
#include "walk_remote.hpp"
#include "configuration.hpp"
#include "options/options.hpp"
#include "robot/humanoid.hpp"
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

    QApplication app(argc, argv);
    walk_remote foo;
    foo.show();
    return app.exec();
}
