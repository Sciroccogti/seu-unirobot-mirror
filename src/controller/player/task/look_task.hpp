#pragma once

#include "task.hpp"
#include "engine/scan/ScanEngine.hpp"

class look_task: public task
{
public:
    look_task(float yaw, float pitch, motion::Head_State state)
        : yaw_(yaw), pitch_(pitch), s_(state), task("look")
    {

    }

    look_task(motion::Head_State state): s_(state), task("look")
    {
        yaw_=0.0;
        pitch_=0.0;
    }

    bool perform()
    {
        motion::SE->set_params(yaw_, pitch_, s_);
        return true;
    }
private:
    float yaw_, pitch_;
    motion::Head_State s_;
};