#include <ctime>
#include "walk_remote.hpp"
#include "configuration.hpp"

using namespace std;
using namespace comm;

walk_remote::walk_remote(): range_(200), scale_d(10), scale_xy(5000),
    client_(CONF.get_config_value<string>(CONF.player()+".address"), CONF.get_config_value<int>("net.tcp.port"))
{
    setMinimumHeight(300);
    dirSlider = new QSlider(Qt::Horizontal);
    dirSlider->setMinimumWidth(200);
    dirSlider->setMinimum(-range_);
    dirSlider->setMaximum(range_);
    dirSlider->setValue(0);

    xSlider = new QSlider(Qt::Vertical);
    xSlider->setMinimumWidth(200);
    xSlider->setMinimum(-range_);
    xSlider->setMaximum(range_);
    xSlider->setValue(0);

    ySlider = new QSlider(Qt::Horizontal);
    ySlider->setMinimumWidth(200);
    ySlider->setMinimum(-range_);
    ySlider->setMaximum(range_);
    ySlider->setValue(0);
    dirLab = new QLabel("d:");
    xLab = new QLabel("x:");
    yLab = new QLabel("y:");
    dirLab->setFixedWidth(80);
    xLab->setFixedWidth(80);
    yLab->setFixedWidth(80);

    hSpinBox = new QDoubleSpinBox();
    startCheck = new QCheckBox("Start");
    btnMan = new QRadioButton("Manual");
    btnRand = new QRadioButton("Random");
    btnSpot = new QRadioButton("Mark Time");
    btnSpot->setMaximumWidth(100);
    btnMan->setChecked(true);

    hSpinBox->setRange(0, 0.05);
    hSpinBox->setDecimals(3);
    hSpinBox->setSingleStep(0.005);
    hSpinBox->setValue(0.03);

    QHBoxLayout *midLayout = new QHBoxLayout();
    QVBoxLayout *mlLayout = new QVBoxLayout();
    mlLayout->addWidget(dirLab);
    mlLayout->addWidget(xLab);
    mlLayout->addWidget(yLab);
    QVBoxLayout *mrLayout = new QVBoxLayout();
    QHBoxLayout *hLayout = new QHBoxLayout();
    hLayout->addWidget(new QLabel("Lift Height"));
    hLayout->addWidget(hSpinBox);
    mrLayout->addLayout(hLayout);
    mrLayout->addWidget(startCheck);
    mrLayout->addWidget(btnMan);
    mrLayout->addWidget(btnSpot);
    mrLayout->addWidget(btnRand);

    midLayout->addLayout(mlLayout);
    midLayout->addWidget(xSlider);
    midLayout->addLayout(mrLayout);

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->addWidget(dirSlider);
    mainLayout->addLayout(midLayout);
    mainLayout->addWidget(ySlider);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    net_info = QString::fromStdString(CONF.get_config_value<string>(CONF.player()+".address"))
               +":"+ QString::number(CONF.get_config_value<int>("net.tcp.port"));
    setWindowTitle(net_info + "(offline)");
    timer= new QTimer;
    timer->start(1000);

    connect(timer, &QTimer::timeout, this, &walk_remote::procTimer);
    connect(dirSlider, &QSlider::valueChanged, this, &walk_remote::procDSlider);
    connect(xSlider, &QSlider::valueChanged, this, &walk_remote::procXSlider);
    connect(ySlider, &QSlider::valueChanged, this, &walk_remote::procYSlider);

    first_connect = true;
    _x = 0;
    _y = 0;
    _dir = 0;
    _h = 0;
    srand(int(time(0)));
    client_.start();
    setEnabled(false);
}

void walk_remote::procTimer()
{
    if(btnRand->isChecked())
    {

        _x = (rand()%(2*range_+1)-range_)/scale_xy;
        _y = (rand()%(2*range_+1)-range_)/scale_xy;
        _dir = (rand()%(2*range_+1)-range_)/scale_d;
    }
    else if(btnSpot->isChecked())
    {
        _x = 0;
        _y = 0;
        _dir = 0;
    }
    else if(btnMan->isChecked())
    {
        _x = xSlider->value()/scale_xy;
        _y = ySlider->value()/scale_xy;
        _dir = dirSlider->value()/scale_d;
    }
    _h = hSpinBox->value();
    updateLab();
    if(client_.is_connected())
    {
        if(first_connect)
        {
            client_.regist(tcp_packet::REMOTE_DATA, tcp_packet::DIR_SUPPLY);
            first_connect = false;
        }
        else
        {
            if(startCheck->isChecked())
            {
                tcp_packet::remote_data_type rtp = tcp_packet::WALK_DATA;
                tcp_packet::tcp_command cmd;
                cmd.type = tcp_packet::REMOTE_DATA;
                cmd.size = sizeof(tcp_packet::remote_data_type)+sizeof(float)*4;
                string data_s;
                data_s.clear();
                data_s.append((char*)(&rtp), sizeof(tcp_packet::remote_data_type));
                data_s.append((char*)(&_x), sizeof(float));
                data_s.append((char*)(&_y), sizeof(float));
                data_s.append((char*)(&_dir), sizeof(float));
                data_s.append((char*)(&_h), sizeof(float));
                memcpy(cmd.data, data_s.data(), cmd.size);
                client_.write(cmd);
            }
        }
        setEnabled(true);
        statusBar()->setStyleSheet("background-color:green");
        setWindowTitle(net_info + "(online)");
    }
    else
    {
        first_connect = true;
        setEnabled(false);
        statusBar()->setStyleSheet("background-color:red");
        setWindowTitle(net_info + "(offline)");
    }
}

void walk_remote::updateLab()
{
    dirLab->setText("d: "+ QString::number(_dir, 'f', 4));
    xLab->setText("x: "+ QString::number(_x, 'f', 4));
    yLab->setText("y: "+ QString::number(_y, 'f', 4));
    update();
}

void walk_remote::procDSlider(int v)
{
    _dir = v/scale_d;
    updateLab();
}

void walk_remote::procXSlider(int v)
{
    _x = v/scale_xy;
    updateLab();
}

void walk_remote::procYSlider(int v)
{
    _y = v/scale_xy;
    updateLab();
}

void walk_remote::closeEvent(QCloseEvent *event)
{
    client_.close();
}