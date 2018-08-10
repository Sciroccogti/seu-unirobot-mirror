#include <QApplication>
#include "walk_remote.hpp"
#include "configuration.hpp"
#include "options/options.hpp"
#include "robot/humanoid.hpp"

using namespace std;
using namespace robot;

int main(int argc, char **argv)
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

    QApplication app(argc, argv);
    walk_remote foo;
    foo.show();
    return app.exec();
}
