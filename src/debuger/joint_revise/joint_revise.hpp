#ifndef SEU_UNIROBOT_DEBUGER_JOINT_REVISE_HPP
#define SEU_UNIROBOT_DEBUGER_JOINT_REVISE_HPP

#include <QtWidgets>
#include "robot/humanoid.hpp"
#include "tcp_client/tcp_client_handler.hpp"

class JSlider:public QWidget
{
    Q_OBJECT
public:
    JSlider(robot::joint_ptr j);
    void reset();
public slots:
    void procSliderChanged(int v);
signals:
    void valueChanged(int id, float v);

private:
    QSlider *slider;
    QLabel *nameLab, *dataLab;
    std::string name_;
    int id_;
    int range_;
    float scale_;
};

class joint_revise: public QMainWindow
{
    Q_OBJECT
public:
    joint_revise();
public slots:
    void procBtnReset();
    void procBtnSave();
    void procValueChanged(int id, float v);
    void procTimer();

protected:
    void closeEvent(QCloseEvent *event);
private:
    std::map<std::string, JSlider*> j_sliders_;
    QPushButton *btnReset, *btnSave;
    QTimer *timer;
    tcp_client_handler client_;
    QString net_info;
    bool first_connect;
};

#endif