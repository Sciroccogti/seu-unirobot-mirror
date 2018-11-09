#pragma once

#include <eigen3/Eigen/Dense>
#include <cmath>

namespace robot_math
{

    inline double deg2rad(const double &x)
    {
        return x * M_PI / 180.0;
    }

    inline double rad2deg(const double &x)
    {
        return x * 180.0 / M_PI;
    }

    inline bool is_zero(const double &x)
    {
        return fabs(x) < 1E-4;
    }

    template <typename T>
    inline T sign(const T &x)
    {
        return (x >= 0) ? 1 : -1;
    }

    template <typename T>
    inline void bound(const T &min, const T &max, T &x)
    {
        if (max < min)
        {
            return;
        }

        if (x < min)
        {
            x = min;
        }

        if (x > max)
        {
            x = max;
        }
    }

    inline Eigen::Matrix3d RotY(const double &rad)
    {
        Eigen::Matrix3d m;
        m << cos(rad),     0.0, sin(rad),
        0.0,        1.0, 0.0,
        -sin(rad),    0.0, cos(rad);
        return m;
    }

    inline Eigen::Matrix3d RotX(const double &rad)
    {
        Eigen::Matrix3d m;
        m << 1.0, 0.0,       0.0,
        0.0, cos(rad),    -sin(rad),
        0.0, sin(rad),    cos(rad);
        return m;
    }

    inline Eigen::Matrix3d RotZ(const double &rad)
    {
        Eigen::Matrix3d m;
        m << cos(rad), -sin(rad),    0.0,
        sin(rad), cos(rad),     0.0,
        0.0,    0.0,        1.0;
        return m;
    }
}

