#pragma once

#include <QtWidgets>
#include "robot/humanoid.hpp"
#include "tcp_client/tcp_client.hpp"

class JSSlider: public QWidget
{
    Q_OBJECT
public:
    JSSlider(robot::joint_ptr j)
        : name_(j->name_), id_(j->jid_), range_(180)
    {
        nameLab = new QLabel(QString::fromStdString(name_));
        nameLab->setFixedWidth(100);
        slider = new QSlider(Qt::Horizontal);
        slider->setMinimumWidth(200);
        slider->setMaximum(range_);
        slider->setMinimum(-range_);
        slider->setValue(0);
        dataLab = new QLabel(QString::number(0));
        dataLab->setFixedWidth(40);
        QHBoxLayout *mainLayout = new QHBoxLayout;
        mainLayout->addWidget(nameLab);
        mainLayout->addWidget(slider);
        mainLayout->addWidget(dataLab);
        setLayout(mainLayout);
        connect(slider, &QSlider::valueChanged, this, &JSSlider::procSliderChanged);
    }

    void reset()
    {
        slider->setValue(0);
        procSliderChanged(0);
    }
public slots:
    void procSliderChanged(int v)
    {
        dataLab->setText(QString::number(v));
        emit valueChanged(id_, (float)v);
    }
signals:
    void valueChanged(int id, float v);

private:
    QSlider *slider;
    QLabel *nameLab, *dataLab;
    std::string name_;
    int id_;
    int range_;
};

class joint_setter: public QMainWindow
{
    Q_OBJECT
public:
    joint_setter(tcp_client &client, QString netinfo, QWidget *parent = nullptr);
public slots:
    void procValueChanged(int id, float v);
    void procTimer();
protected:
    virtual void keyPressEvent(QKeyEvent *event);

private:
    std::map<std::string, JSSlider *> j_sliders_;
    QTimer *timer;
    tcp_client &client_;
    QString net_info;
    bool first_connect;
};
