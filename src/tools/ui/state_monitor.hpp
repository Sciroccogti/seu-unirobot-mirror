#pragma once

#include <QtWidgets>
#include <map>
#include <string>
#include "model.hpp"

class StateMonitor: public QMainWindow
{
    Q_OBJECT
public:
    StateMonitor(int id);
    void update_state(int s);
private:
    std::map<int, QLabel*> state_labels_;
    int state_;
    int id_;
};
