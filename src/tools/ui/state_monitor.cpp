#include "state_monitor.hpp"

using namespace std;

StateMonitor::StateMonitor(int id): id_(id)
{
    int s_count = fsm_state_infos.size();
    const int c=4;
    int i=0;
    QHBoxLayout *mainLayout = new QHBoxLayout;
    QVBoxLayout *stateLayout;
    for(auto &f: fsm_state_infos)
    {
        if(i%4==0)
        {
            stateLayout = new QVBoxLayout;
            mainLayout->addLayout(stateLayout);
        }
        QLabel *label = new QLabel(QString::fromStdString(f.second));
        label->setFrameShape (QFrame::Box);
        label->setMinimumHeight(30);
        label->setAlignment(Qt::AlignCenter);
        label->setStyleSheet("border-width: 1px;border-style: solid;  background-color: #FF6633; color: rgb(255, 255, 255)");
        state_labels_[f.first] = label;
        stateLayout->addWidget(label);
        i++;
    }
    QWidget *mainWidget = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);
    setWindowTitle(QString::number(id));
}

void StateMonitor::update_state(int s)
{
    state_ = s;
    for(auto &sl: state_labels_)
    {
        if(sl.first != state_)
            sl.second->setStyleSheet("background-color: #FF6633;");
        else
            sl.second->setStyleSheet("background-color: #66CC33;");
    }
    this->update();
}