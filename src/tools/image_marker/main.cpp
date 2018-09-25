#include <QApplication>
#include "image_marker.hpp"

using namespace std;

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    image_marker foo;
    foo.show();
    return app.exec();
}
