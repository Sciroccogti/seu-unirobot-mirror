#pragma once

#include "plan.hpp"
#include "walk/WalkEngine.hpp"
#include "joints_plan.hpp"

class walk_plan: public plan
{
public:
    walk_plan(const float &x, const float &y, const float &dir, const float &h = 0.04)
        : plan("walk_plan", "body")
    {
        params_[0] = x;
        params_[1] = y;
        params_[2] = dir;
        params_[3] = h;
    }

    bool perform()
    {
        walk::WE->set_params(params_);
        MADT->mode() = adapter::MODE_WALK;
        return true;
    }
private:
    Eigen::Vector4f params_;
};

