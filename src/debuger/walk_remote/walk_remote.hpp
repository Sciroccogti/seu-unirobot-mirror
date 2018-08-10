#ifndef SEU_UNIROBOT_DEBUGER_WALK_REMOTE_HPP
#define SEU_UNIROBOT_DEBUGER_WALK_REMOTE_HPP

#include <QtWidgets>
#include "tcp_client/tcp_client_handler.hpp"

class walk_remote : public QMainWindow
{
    Q_OBJECT
public:
    walk_remote();
    void updateLab();
public slots:
    void procTimer();
    void procXSlider(int v);
    void procYSlider(int v);
    void procDSlider(int v);

protected:
    void closeEvent(QCloseEvent *event);
private:
    QDoubleSpinBox *hSpinBox;
    QCheckBox *startCheck;
    QRadioButton *btnSpot, *btnRand;
    QSlider *dirSlider, *xSlider, *ySlider;
    QLabel *dirLab, *xLab, *yLab;
    float _x, _y, _dir, _h;
    int range_;
    float scale_d;
    float scale_xy;
    QTimer *timer;
    tcp_client_handler client_;
    QString net_info;
    bool first_connect;
};

#endif