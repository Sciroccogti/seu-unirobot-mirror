#include "joint_setter.hpp"
#include "configuration.hpp"

using namespace robot;
using namespace std;

joint_setter::joint_setter(tcp_client &client, QString netinfo, QWidget *parent)
    : net_info(netinfo), client_(client), QMainWindow(parent)
{
    setAttribute(Qt::WA_DeleteOnClose);
    QHBoxLayout *mainLayout = new QHBoxLayout();
    QVBoxLayout *leftLayout = new QVBoxLayout();
    QVBoxLayout *rightLayout = new QVBoxLayout();

    first_connect = true;
    JSSlider *slider;

    for (auto &jo : ROBOT->get_joint_map())
    {
        slider = new JSSlider(jo.second);
        connect(slider, &JSSlider::valueChanged, this, &joint_setter::procValueChanged);

        if (jo.first == "jhead2" || jo.first.find("jr") != string::npos)
        {
            rightLayout->addWidget(slider);
        }
        else
        {
            leftLayout->addWidget(slider);
        }

        j_sliders_[jo.first] = slider;
    }

    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);
    setWindowTitle(net_info);

    timer = new QTimer;
    timer->start(1000);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    connect(timer, &QTimer::timeout, this, &joint_setter::procTimer);
    setEnabled(false);
}

void joint_setter::procTimer()
{
    if (client_.is_connected())
    {
        if (first_connect)
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

void joint_setter::procValueChanged(int id, float v)
{
    ROBOT->get_joint(id)->set_deg(v);
    tcp_command cmd;
    remote_data_type t = JOINT_SET;
    cmd.data.clear();
    cmd.type = REMOTE_DATA;
    cmd.size = sizeof(robot_joint_deg) * ROBOT->get_joint_map().size() + enum_size;
    cmd.data.clear();
    robot_joint_deg offset;
    cmd.data.append((char *)&t, enum_size);

    for (auto &j : ROBOT->get_joint_map())
    {
        offset.id = j.second->jid_;
        offset.deg = j.second->get_deg();
        cmd.data.append((char *)(&offset), sizeof(robot_joint_deg));
    }

    client_.write(cmd);
}

void joint_setter::keyPressEvent(QKeyEvent *event)
{
    switch (event->key())
    {
        case Qt::Key_Space:
            for (auto &s : j_sliders_)
            {
                s.second->reset();
                ROBOT->get_joint_map()[s.first]->set_deg(0.0);
            }
            procValueChanged(10, 0);
            break;
        default:
            break;
    }
}