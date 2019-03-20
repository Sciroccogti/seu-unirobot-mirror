#pragma once

#include "matrix.hpp"

namespace robot_math
{
    inline Eigen::Matrix2d rotation_mat_2d(const double &deg)
    {
        Eigen::Matrix2d temp;
            double rad = deg2rad(deg);
            temp<<std::cos(rad), std::sin(rad),
                  -std::sin(rad), std::cos(rad);
            return temp;
    }
}
