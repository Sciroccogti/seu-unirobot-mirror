#include "camera_calibration.hpp"
#include <QApplication>

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    camera_calibration foo;
    foo.show();
    return app.exec();
}