#include "image_monitor.hpp"
#include "configuration.hpp"

using namespace std;
using namespace robot;
using namespace comm;

image_monitor::image_monitor()
    :client_(CONF.get_config_value<string>(CONF.player()+".address"), CONF.get_config_value<int>("net.tcp.port"),
            bind(&image_monitor::data_handler, this, placeholders::_1, placeholders::_2, placeholders::_3))
{
    first_connect = true;
    imageLab = new ImageLabel(640, 480);
    
    pitchSlider = new QSlider(Qt::Vertical);
    pitchSlider->setRange(-90, 90);
    yawSlider = new QSlider(Qt::Horizontal);
    yawSlider->setRange(-90, 90);
    
    QHBoxLayout *upLayout = new QHBoxLayout();
    upLayout->addWidget(imageLab);
    upLayout->addWidget(pitchSlider);
    
    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->addLayout(upLayout);
    mainLayout->addWidget(yawSlider);
    
    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);
    
    yawLab = new QLabel();
    pitchLab = new QLabel();
    netLab = new QLabel();
    netLab->setFixedWidth(100);
    
    statusBar()->addWidget(pitchLab);
    statusBar()->addWidget(yawLab);
    statusBar()->addWidget(netLab);
    
    net_info = QString::fromStdString(CONF.get_config_value<string>(CONF.player()+".address"))
               +":"+ QString::number(CONF.get_config_value<int>("net.tcp.port"));
    setWindowTitle(net_info + "(offline)");

    timer= new QTimer;
    timer->start(30);

    connect(timer, SIGNAL(timeout()), this, SLOT(procTimer()));
    connect(yawSlider, SIGNAL(valueChanged(int)), this, SLOT(procYawSlider(int)));
    connect(pitchSlider, SIGNAL(valueChanged(int)), this, SLOT(procPitchSlider(int)));
    client_.start();
}

void image_monitor::data_handler(const char *data, const int &size, const int &type)
{
    if(type == tcp_packet::IMAGE_DATA)
    {
        imageLab->set_image((uint8_t*)data, size);
    }
}

void image_monitor::procTimer()
{
    if(client_.is_connected())
    {
        if(first_connect)
        {
            client_.regist(tcp_packet::IMAGE_DATA, tcp_packet::DIR_APPLY);
            usleep(10000);
            client_.regist(tcp_packet::JOINT_DATA, tcp_packet::DIR_SUPPLY);
        }
        first_connect = false;
        netLab->setStyleSheet("background-color:green");
        setWindowTitle(net_info + "(online)");
    }
    else
    {
        first_connect = true;
        netLab->setStyleSheet("background-color:red");
        setWindowTitle(net_info + "(offline)");
    }
}

void image_monitor::procPitchSlider(int v)
{
    pitchLab->setText(QString::number(v));
}

void image_monitor::procYawSlider(int v)
{
    pitchLab->setText(QString::number(v));
}


void image_monitor::closeEvent(QCloseEvent *event)
{
    client_.close();
}