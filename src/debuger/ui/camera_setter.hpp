#ifndef SEU_UNIROBOT_DEBUGER_CAMERA_SETTER_HPP
#define SEU_UNIROBOT_DEBUGER_CAMERA_SETTER_HPP

#include <QtWidgets>
#include "model.hpp"
#include "tcp_client/tcp_client.hpp"

class CtrlSlider:public QWidget
{
    Q_OBJECT
public:
    CtrlSlider(const camera_ctrl_info &info)
        : info_(info)
    {
        qRegisterMetaType<camera_ctrl_info>("ctrl_item_info");
        nameLab = new QLabel(QString::fromStdString(std::string((char*)(info.qctrl.name))));
        slider = new QSlider(Qt::Horizontal);
        slider->setMinimumWidth(200);
        slider->setMaximum(info.qctrl.maximum);
        slider->setMinimum(info.qctrl.minimum);
        slider->setValue(info.ctrl.value);
        dataLab = new QLabel(QString::number(info.ctrl.value));
        dataLab->setFixedWidth(40);
        desLab = new QLabel(QString::fromStdString(info.menu));
        QHBoxLayout *sLayout = new QHBoxLayout;
        sLayout->addWidget(slider);
        sLayout->addWidget(dataLab);
        QVBoxLayout *mainLayout = new QVBoxLayout;
        mainLayout->addWidget(nameLab);
        mainLayout->addLayout(sLayout);
        mainLayout->addWidget(desLab);
        setLayout(mainLayout);
        connect(slider, &QSlider::valueChanged, this, &CtrlSlider::procSliderChanged);
    }
    
    void reset()
    {
        slider->setValue(info_.qctrl.default_value);
        procSliderChanged(info_.qctrl.default_value);
    }
    
    camera_ctrl_info c_info() const { return info_; }
public slots:
    void procSliderChanged(int v)
    {
        dataLab->setText(QString::number(v));
        info_.ctrl.value = v;
        emit valueChanged(info_);
    }
signals:
    void valueChanged(camera_ctrl_info info);

private:
    camera_ctrl_info info_;
    QSlider *slider;
    QLabel *nameLab, *dataLab, *desLab;
};

class camera_setter: public QMainWindow
{
    Q_OBJECT
public:
    camera_setter(tcp_client &client, QString netinfo, QWidget *parent=nullptr);
public slots:
    void procBtnReset();
    void procBtnSave();
    void procValueChanged(camera_ctrl_info info);
    void procTimer();

private:
    std::vector<CtrlSlider*> c_sliders_;
    std::vector<camera_ctrl_info> ctrl_items_;
    QPushButton *btnReset, *btnSave;
    QTimer *timer;
    tcp_client &client_;
    QString net_info;
    bool first_connect;
};

#endif