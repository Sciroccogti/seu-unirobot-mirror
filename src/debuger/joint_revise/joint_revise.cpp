#include "joint_revise.hpp"
#include "configuration.hpp"

using namespace robot;
using namespace std;
using namespace comm;

JSlider::JSlider(joint_ptr j): name_(j->name_), id_(j->jid_), range_(100), scale_(10.0)
{
    nameLab = new QLabel(QString::fromStdString(name_));
    nameLab->setFixedWidth(100);
    slider = new QSlider(Qt::Horizontal);
    slider->setMinimumWidth(200);
    slider->setMaximum(range_);
    slider->setMinimum(-range_);
    slider->setValue(static_cast<int>(scale_*j->offset_));
    dataLab = new QLabel(QString::number(j->offset_, 'f', 1));
    dataLab->setFixedWidth(40);
    QHBoxLayout *mainLayout = new QHBoxLayout;
    mainLayout->addWidget(nameLab);
    mainLayout->addWidget(slider);
    mainLayout->addWidget(dataLab);
    setLayout(mainLayout);
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(procSliderChanged(int)));
}

void JSlider::reset()
{
    slider->setValue(0);
    procSliderChanged(0);
}

void JSlider::procSliderChanged(int v)
{
    float offset = v/scale_;
    dataLab->setText(QString::number(offset, 'f', 1));
    emit valueChanged(id_, offset);
}

joint_revise::joint_revise()
    :client_(CONF.get_config_value<string>(CONF.player()+".address"), CONF.get_config_value<int>("tcp.port"))
{
    QHBoxLayout *mainLayout = new QHBoxLayout();
    QVBoxLayout *leftLayout = new QVBoxLayout();
    QVBoxLayout *rightLayout = new QVBoxLayout();

    first_connect = true;
    JSlider *slider;
    for(auto jo:ROBOT.get_joint_map())
    {
        slider = new JSlider(jo.second);
        connect(slider, SIGNAL(valueChanged(int, float)), this, SLOT(procValueChanged(int, float)));
        if(jo.first == "jhead2" || jo.first.find("jr") != string::npos)
            rightLayout->addWidget(slider);
        else leftLayout->addWidget(slider);
        j_sliders_[jo.first] = slider;
    }

    btnReset = new QPushButton("Reset");
    btnSave = new QPushButton("Save");
    leftLayout->addWidget(btnReset);
    rightLayout->addWidget(btnSave);
    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);
    net_info = QString::fromStdString(CONF.get_config_value<string>(CONF.player()+".address"))
               +":"+ QString::number(CONF.get_config_value<int>("tcp.port"));
    setWindowTitle(net_info + "(offline)");

    timer= new QTimer;
    timer->start(1000);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    connect(btnReset, SIGNAL(clicked(bool)), this, SLOT(procBtnReset()));
    connect(btnSave, SIGNAL(clicked(bool)), this, SLOT(procBtnSave()));
    connect(timer, SIGNAL(timeout()), this, SLOT(procTimer()));
    client_.start();
    setEnabled(false);
}

void joint_revise::procTimer()
{
    if(client_.is_connected())
    {
        if(first_connect)
            client_.regist(tcp_packet::JOINT_OFFSET);
        first_connect = false;
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

void joint_revise::procBtnReset()
{
    for(auto s:j_sliders_)
    {
        s.second->reset();
        ROBOT.get_joint_map()[s.first]->offset_ = 0.0;
    }
}

void joint_revise::procBtnSave()
{

}

void joint_revise::procValueChanged(int id, float v)
{
    ROBOT.get_joint(id)->offset_ = v;
    tcp_packet::tcp_command cmd;
    cmd.type = tcp_packet::JOINT_OFFSET;
    cmd.size = sizeof(robot_joint_offset)*ROBOT.get_joint_map().size();
    robot_joint_offset offset;
    int i=0;
    for(auto j:ROBOT.get_joint_map())
    {
        offset.id = j.second->jid_;
        offset.offset = j.second->offset_;
        memcpy(cmd.data+i* sizeof(robot_joint_offset), (char*)(&offset), sizeof(robot_joint_offset));
        i++;
    }
    client_.write(cmd);
}

void joint_revise::closeEvent(QCloseEvent *event)
{
    client_.close();
}

joint_revise::~joint_revise()
{
}