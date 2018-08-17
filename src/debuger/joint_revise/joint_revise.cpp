#include "joint_revise.hpp"
#include "configuration.hpp"

using namespace robot;
using namespace std;
using namespace comm;

joint_revise::joint_revise()
    :client_(CONF.get_config_value<string>(CONF.player()+".address"), CONF.get_config_value<int>("net.tcp.port"))
{
    QHBoxLayout *mainLayout = new QHBoxLayout();
    QVBoxLayout *leftLayout = new QVBoxLayout();
    QVBoxLayout *rightLayout = new QVBoxLayout();

    first_connect = true;
    JSlider *slider;
    for(auto jo:ROBOT.get_joint_map())
    {
        slider = new JSlider(jo.second);
        connect(slider, &JSlider::valueChanged, this, &joint_revise::procValueChanged);
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
               +":"+ QString::number(CONF.get_config_value<int>("net.tcp.port"));
    setWindowTitle(net_info + "(offline)");

    timer= new QTimer;
    timer->start(1000);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    connect(btnReset, &QPushButton::clicked, this, &joint_revise::procBtnReset);
    connect(btnSave, &QPushButton::clicked, this, &joint_revise::procBtnSave);
    connect(timer, &QTimer::timeout, this, &joint_revise::procTimer);
    client_.start();
    setEnabled(false);
}

void joint_revise::procTimer()
{
    if(client_.is_connected())
    {
        if(first_connect)
        {
            client_.regist(tcp_packet::JOINT_DATA, tcp_packet::DIR_SUPPLY);
            first_connect = false;
        }
        else
        {
            tcp_packet::tcp_command cmd;
            cmd.type = tcp_packet::JOINT_DATA;
            cmd.size = sizeof(robot_joint_deg)*ROBOT.get_joint_map().size();
            robot_joint_deg offset;
            int i=0;
            for(auto j:ROBOT.get_joint_map())
            {
                offset.id = j.second->jid_;
                offset.deg = j.second->offset_;
                memcpy(cmd.data+i* sizeof(robot_joint_deg), (char*)(&offset), sizeof(robot_joint_deg));
                i++;
            }
            client_.write(cmd);
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
}

void joint_revise::closeEvent(QCloseEvent *event)
{
    client_.close();
}
