#include "camera_setter.hpp"
#include "configuration.hpp"
#include "parser/camera_parser.hpp"

using namespace std;

camera_setter::camera_setter(tcp_client &client, QString netinfo, QWidget *parent)
    :client_(client), net_info(netinfo), QMainWindow(parent)
{
    setAttribute(Qt::WA_DeleteOnClose);
    QHBoxLayout *mainLayout = new QHBoxLayout();
    QVBoxLayout *leftLayout = new QVBoxLayout();
    QVBoxLayout *rightLayout = new QVBoxLayout();

    first_connect = true;
    CtrlSlider *slider;
    
    parser::camera_parser::parse(CONF->get_config_value<string>(CONF->player()+".camera_file"), ctrl_items_);
    int i=0;
    for(auto &it: ctrl_items_)
    {
        slider = new CtrlSlider(it);
        connect(slider, &CtrlSlider::valueChanged, this, &camera_setter::procValueChanged);
        c_sliders_[it.id] = slider;
        if(i<ctrl_items_.size()/2)
            leftLayout->addWidget(slider);
        else
            rightLayout->addWidget(slider);
        i++;
    }

    btnReset = new QPushButton("Reset");
    btnSave = new QPushButton("Save");
    leftLayout->addWidget(btnReset);
    rightLayout->addWidget(btnSave);
    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);
    setWindowTitle(net_info);

    timer= new QTimer;
    timer->start(1000);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    connect(btnReset, &QPushButton::clicked, this, &camera_setter::procBtnReset);
    connect(btnSave, &QPushButton::clicked, this, &camera_setter::procBtnSave);
    connect(timer, &QTimer::timeout, this, &camera_setter::procTimer);
    setEnabled(false);
}

void camera_setter::procTimer()
{
    if(client_.is_connected())
    {
        if(first_connect)
        {
            client_.regist(REMOTE_DATA, DIR_SUPPLY);
            first_connect = false;
        }
        setEnabled(true);
        statusBar()->setStyleSheet("background-color:green");
    }
    else
    {
        first_connect = true;
        setEnabled(false);
        statusBar()->setStyleSheet("background-color:red");
    }
}

void camera_setter::procBtnReset()
{
    
}

void camera_setter::procBtnSave()
{

}

void camera_setter::procValueChanged(int id, float v)
{
    /*
    ROBOT->get_joint(id)->offset_ = v;
    tcp_command cmd;
    remote_data_type t = JOINT_OFFSET;
    cmd.data.clear();
    cmd.type = REMOTE_DATA;
    cmd.size = sizeof(robot_joint_deg)*ROBOT->get_joint_map().size()+rmt_type_size;
    cmd.data.clear();
    robot_joint_deg offset;
    cmd.data.append((char*)&t, rmt_type_size);
    for(auto &j:ROBOT->get_joint_map())
    {
        offset.id = j.second->jid_;
        offset.deg = j.second->offset_;
        cmd.data.append((char*)(&offset), sizeof(robot_joint_deg));
    }
    client_.write(cmd);
    */
}
