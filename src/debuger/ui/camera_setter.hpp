#ifndef SEU_UNIROBOT_DEBUGER_CAMERA_SETTER_HPP
#define SEU_UNIROBOT_DEBUGER_CAMERA_SETTER_HPP

#include <QtWidgets>
#include "model.hpp"
#include "tcp_client/tcp_client.hpp"

class CtrlSlider:public QWidget
{
    Q_OBJECT
public:
    CtrlSlider(const ctrl_item_info &info, const int &min=0, const int &max=1000)
        : info_(info)
    {
        nameLab = new QLabel(QString::fromStdString(info.name));
        slider = new QSlider(Qt::Horizontal);
        slider->setMinimumWidth(200);
        slider->setMaximum(max);
        slider->setMinimum(min);
        slider->setValue(info.value);
        dataLab = new QLabel(QString::number(info.value));
        dataLab->setFixedWidth(40);
        QHBoxLayout *sLayout = new QHBoxLayout;
        sLayout->addWidget(slider);
        sLayout->addWidget(dataLab);
        QVBoxLayout *mainLayout = new QVBoxLayout;
        mainLayout->addWidget(nameLab);
        mainLayout->addLayout(sLayout);
        setLayout(mainLayout);
        connect(slider, &QSlider::valueChanged, this, &CtrlSlider::procSliderChanged);
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
        //emit valueChanged(id_, offset);
    }
signals:
    void valueChanged(int id, float v);

private:
    ctrl_item_info info_;
    QSlider *slider;
    QLabel *nameLab, *dataLab;
};

class camera_setter: public QMainWindow
{
    Q_OBJECT
public:
    camera_setter(tcp_client &client, QString netinfo, QWidget *parent=nullptr);
public slots:
    void procBtnReset();
    void procBtnSave();
    void procValueChanged(int id, float v);
    void procTimer();

private:
    std::map<unsigned int, CtrlSlider*> c_sliders_;
    std::vector<ctrl_item_info> ctrl_items_;
    QPushButton *btnReset, *btnSave;
    QTimer *timer;
    tcp_client &client_;
    QString net_info;
    bool first_connect;
};

#endif