//
// Created by lcseu on 18-8-11.
//

#ifndef SEU_UNIROBOT_DEBUGER_ACTION_MONITOR_HPP
#define SEU_UNIROBOT_DEBUGER_ACTION_MONITOR_HPP

#include <QtWidgets>
#include "module/RobotGL.hpp"
#include "robot/humanoid.hpp"
#include "tcp_client/tcp_client_handler.hpp"

class action_monitor: public QMainWindow
{
    Q_OBJECT
public:
    action_monitor();
    void data_handler(const char *data, const int &size, const int &type);
public slots:
    void procTimer();

protected:
    void closeEvent(QCloseEvent *event);
private:
    RobotGL *rgl;
    QTimer *timer;
    tcp_client_handler client_;
    QString net_info;
    bool first_connect;
};

#endif //SEU_UNIROBOT_ACTION_MONITOR_HPP
