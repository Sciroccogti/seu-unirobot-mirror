#pragma once

#include "task.hpp"
#include "engine/scan/ScanEngine.hpp"
#include <vector>
#include <eigen3/Eigen/Dense>

class look_task: public task
{
public:
    look_task(float yaw, float pitch)
        :s_(motion::HEAD_STATE_LOOKAT), task("look")
    {
        pitch_ = pitch;
        yaw_ = yaw;
    }

    look_task(motion::Head_State state): s_(state), task("look")
    {
        pitch_ = 0.0;
        yaw_ = 0.0;
    }

    bool perform()
    {
        motion::SE->set_params(yaw_, pitch_, s_);
        return true;
    }
private:
    float pitch_, yaw_;
    motion::Head_State s_;
};
