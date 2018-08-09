#include <QApplication>
#include "static_action.hpp"
using namespace std;

int main(int argc, char **argv)
{
    int id;
    if(argc<2)
    {
        cout<<"\033[31mplease run with ./static_action id\033[0m"<<endl;
        exit(1);
    }
    try
    {
        id = std::atoi(argv[1]);
    }
    catch(std::exception &e)
    {
        cout<<"argument wrong"<<endl;
        exit(1);
    }
    QApplication app(argc, argv);
    glutInit(&argc, argv);
    static_action foo(id);
    foo.showMaximized();
    foo.show();
    return app.exec();
}
