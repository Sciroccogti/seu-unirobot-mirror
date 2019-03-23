#pragma once

#include "task.hpp"
#include "engine/scan/ScanEngine.hpp"

class look_task: public task
{
public:
    look_task(float yaw, float pitch, bool enable=false)
        : yaw_(yaw), pitch_(pitch), e_(enable), task("look")
    {

    }

    look_task(bool enable): e_(enable), task("scan")
    {
        yaw_=0.0;
        pitch_=0.0;
    }

    bool perform()
    {
        motion::SE->set_params(yaw_, pitch_, e_);
        return true;
    }
private:
    float yaw_, pitch_;
    bool e_;
};