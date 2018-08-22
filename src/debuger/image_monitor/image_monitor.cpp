#include "image_monitor.hpp"
#include "configuration.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace robot;

image_monitor::image_monitor()
    :client_(CONF.get_config_value<string>(CONF.player()+".address"), CONF.get_config_value<int>("net.tcp.port"),
            bind(&image_monitor::data_handler, this, placeholders::_1))
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
    setWindowTitle(net_info);

    timer= new QTimer;
    timer->start(1000);

    connect(timer, &QTimer::timeout, this, &image_monitor::procTimer);
    connect(yawSlider, &QSlider::valueChanged, this, &image_monitor::procYawSlider);
    connect(pitchSlider, &QSlider::valueChanged, this, &image_monitor::procPitchSlider);;
    client_.start();
}

void image_monitor::data_handler(const tcp_command cmd)
{
    static int count=0;
    //cout<<"recv: "<<++count<<endl;
    if(cmd.type == IMAGE_DATA)
    {
        vector<unsigned char> buf(cmd.size);
        memcpy(&buf[0], cmd.data.c_str(), cmd.size);
        Mat bgr = imdecode(buf, cv::IMREAD_COLOR);
        Mat dst;
        cvtColor(bgr, dst, CV_BGR2RGB);
        QImage *disImage = new QImage((const unsigned char*)(dst.data),dst.cols,dst.rows,QImage::Format_RGB888);
        imageLab->setPixmap(QPixmap::fromImage(disImage->scaled(imageLab->size(), Qt::KeepAspectRatio)));
        delete disImage;
        bgr.release();
        dst.release();
    }
}

void image_monitor::procTimer()
{
    if(client_.is_connected())
    {
        if(first_connect)
        {
            client_.regist(IMAGE_DATA, DIR_APPLY);
            //usleep(10000);
            //client_.regist(JOINT_DATA, DIR_SUPPLY);
        }
        first_connect = false;
        netLab->setStyleSheet("background-color:green");
    }
    else
    {
        first_connect = true;
        netLab->setStyleSheet("background-color:red");
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
    client_.stop();
}