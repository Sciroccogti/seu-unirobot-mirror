#include <QApplication>
#include "image_debuger.hpp"
#include "configuration.hpp"
#include "options/options.hpp"

using namespace std;
using namespace robot;

int main(int argc, char **argv)
{
    if (!OPTS->init(argc, argv))
    {
        std::cout << "options init failed\n";
        exit(1);
    }

    if (!CONF->init(OPTS->id()))
    {
        std::cout << "config init failed\n";
        exit(2);
    }

    QApplication app(argc, argv);
    ImageDebuger foo;
    foo.show();
    return app.exec();
}
