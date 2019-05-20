#include "camera_setter.hpp"
#include "configuration.hpp"
#include "parser/parser.hpp"

using namespace std;

CameraSetter::CameraSetter(tcp_client &client, QString netinfo, QWidget *parent)
    : client_(client), net_info(netinfo), QMainWindow(parent)
{
    setAttribute(Qt::WA_DeleteOnClose);
    QHBoxLayout *mainLayout = new QHBoxLayout();
    QVBoxLayout *leftLayout = new QVBoxLayout();
    QVBoxLayout *rightLayout = new QVBoxLayout();

    first_connect = true;
    CtrlSlider *slider;

    parser::parse(CONF->get_config_value<string>(CONF->player() + ".camera_info_file"), ctrl_items_);

    for (auto it : ctrl_items_)
    {
        slider = new CtrlSlider(it.second);
        connect(slider, &CtrlSlider::valueChanged, this, &CameraSetter::procValueChanged);
        c_sliders_.push_back(slider);

        if ((it.first.find("gain") != string::npos || it.first.find("sat") != string::npos)
            && it.first.find("exp") == string::npos)
        {
            rightLayout->addWidget(slider);
        }
        else
        {
            leftLayout->addWidget(slider);
        }
    }

    btnReset = new QPushButton("Reset");
    btnSave = new QPushButton("Save");
    leftLayout->addWidget(btnReset);
    rightLayout->addWidget(btnSave);
    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);
    setWindowTitle(net_info);

    timer = new QTimer;
    timer->start(1000);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    connect(btnReset, &QPushButton::clicked, this, &CameraSetter::procBtnReset);
    connect(btnSave, &QPushButton::clicked, this, &CameraSetter::procBtnSave);
    connect(timer, &QTimer::timeout, this, &CameraSetter::procTimer);
    setEnabled(false);
}

void CameraSetter::procTimer()
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

void CameraSetter::procBtnReset()
{
    for (auto &c : ctrl_items_)
    {
        c.second.value = c.second.default_value;
        procValueChanged(c.second);
    }

    for (auto &s : c_sliders_)
    {
        s->reset();
    }
}

void CameraSetter::procBtnSave()
{
    parser::save(CONF->get_config_value<string>(CONF->player() + ".camera_file"), ctrl_items_);
}

void CameraSetter::procValueChanged(camera_info info)
{
    for (auto &item : ctrl_items_)
    {
        if (item.second.name == info.name)
        {
            item.second.value = info.value;
            break;
        }
    }

    tcp_command cmd;
    remote_data_type t = CAMERA_SET;
    cmd.type = REMOTE_DATA;
    cmd.size = int_size * 2 + enum_size;
    cmd.data.clear();
    cmd.data.append((char *)&t, enum_size);
    cmd.data.append((char *) & (info.id), int_size);
    cmd.data.append((char *) & (info.value), int_size);
    client_.write(cmd);
}
