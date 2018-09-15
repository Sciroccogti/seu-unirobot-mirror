#ifndef SEU_UNIROBOT_DEBUGER_WALK_REMOTE_HPP
#define SEU_UNIROBOT_DEBUGER_WALK_REMOTE_HPP

#include <QtWidgets>
#include "tcp_client/tcp_client.hpp"

class walk_remote : public QMainWindow
{
    Q_OBJECT
public:
    walk_remote(tcp_client &client, QString netinfo, QWidget *parent = nullptr);
    void updateLab();
public slots:
    void procTimer();
    void procXSlider(int v);
    void procYSlider(int v);
    void procDSlider(int v);

private:
    QDoubleSpinBox *hSpinBox;
    QCheckBox *startCheck;
    QRadioButton *btnSpot, *btnRand, *btnMan;
    QSlider *dirSlider, *xSlider, *ySlider;
    QLabel *dirLab, *xLab, *yLab;
    float _x, _y, _dir, _h;
    int range_;
    float scale_d;
    float scale_xy;
    QTimer *timer;
    QString netinfo_;
    tcp_client &client_;
    bool first_connect;
};

#endif