#ifndef SEU_UNIROBOT_DEBUGER_JOINT_REVISE_HPP
#define SEU_UNIROBOT_DEBUGER_JOINT_REVISE_HPP

#include <QtWidgets>
#include "robot/robot_define.hpp"

class JSlider:public QWidget
{
    Q_OBJECT
public:
    JSlider(robot::joint_ptr j, const float &deg);
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

class joint_revise: public QDialog
{
    Q_OBJECT
public:
    joint_revise();
    ~joint_revise();
public slots:
    void procBtnReset();
    void procBtnSave();
    void procValueChanged(int id, float v);
private:
    std::map<std::string, JSlider*> j_sliders_;
    QPushButton *btnReset, *btnSave;
};

#endif