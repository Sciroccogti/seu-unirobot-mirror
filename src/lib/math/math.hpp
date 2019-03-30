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

    inline double azimuth(const Eigen::Vector2d &v)
    {
        return rad2deg(std::atan2(v.y(), v.x()));
    }

    template<typename T>
    inline T normalize_deg(T deg)
    {
        while (deg > 180.0)
            deg -= 360.0;
        while (deg < -180.0)
            deg += 360.0;
        return deg;
    }
}
