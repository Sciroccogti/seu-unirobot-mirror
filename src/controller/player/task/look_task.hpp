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
        angles_.push_back(Eigen::Vector2f(yaw, pitch));
    }
    
    look_task(const std::vector<Eigen::Vector2f> &angles, motion::Head_State state)
    {
        angles_ = angles;
        s_ = state;
    }

    look_task(motion::Head_State state): s_(state), task("look")
    {

    }

    bool perform()
    {
        //motion::SE->set_params(yaw_, pitch_, s_);
        return true;
    }
private:
    std::vector<Eigen::Vector2f> angles_;
    motion::Head_State s_;
};
