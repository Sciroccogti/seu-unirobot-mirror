#include "action_monitor.hpp"
#include "configuration.hpp"

using namespace std;
using namespace robot;
using namespace comm;

action_monitor::action_monitor()
    :client_(CONF.get_config_value<string>(CONF.player()+".address"), CONF.get_config_value<int>("tcp.port"),
            std::bind(&action_monitor::data_handler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3))
{
    rgl = new robot_gl(ROBOT.get_main_bone(), ROBOT.get_joint_map());
    setCentralWidget(rgl);

    first_connect = true;
    net_info = QString::fromStdString(CONF.get_config_value<string>(CONF.player()+".address"))
               +":"+ QString::number(CONF.get_config_value<int>("tcp.port"));
    setWindowTitle(net_info + "(offline)");

    timer= new QTimer;
    timer->start(1000);

    connect(timer, SIGNAL(timeout()), this, SLOT(procTimer()));
    client_.start();
}

void action_monitor::data_handler(const char *data, const int &size, const int &type)
{
    if(type == tcp_packet::ACT_DATA)
    {
    }
}

void action_monitor::procTimer()
{
    if(client_.is_connected())
    {
        if(first_connect)
            client_.regist(tcp_packet::ACT_DATA, tcp_packet::DIR_APPLY);
        first_connect = false;
        statusBar()->setStyleSheet("background-color:green");
        setWindowTitle(net_info + "(online)");
    }
    else
    {
        first_connect = true;
        statusBar()->setStyleSheet("background-color:red");
        setWindowTitle(net_info + "(offline)");
    }
}

void action_monitor::closeEvent(QCloseEvent *event)
{
    client_.close();
}