#pragma once

#include <QtWidgets>
#include <unordered_map>
#include <string>
#include "model.hpp"

class StateMonitor: public QMainWindow
{
    Q_OBJECT
public:
    StateMonitor(int id);
    void update_state(int s);
private:
    std::unordered_map<int, QLabel*> state_labels_;
    int state_;
    int id_;
};
