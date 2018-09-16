#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>

namespace robot_math
{
    template<typename T, int N>
    class vector_t: public Eigen::Matrix<T, N, 1>
    {
    public:
        vector_t()
        {

        }
        vector_t(const T &x, const T &y)
        {
            this->x() = x;
            this->y() = y;
        }

        vector_t(const T &x, const T &y, const T &z)
        {
            this->x() = x;
            this->y() = y;
            this->z() = z;
        }

        vector_t(const T &x, const T &y, const T &z, const T &w)
        {
            this->x() = x;
            this->y() = y;
            this->z() = z;
            this->w() = w;
        }

        T x() const
        {
            return this->data()[0];
        }

        T &x()
        {
            return this->data()[0];
        }

        T y() const
        {
            return this->data()[1];
        }

        T &y()
        {
            return this->data()[1];
        }

        T z() const
        {
            return this->data()[2];
        }

        T &z()
        {
            return this->data()[2];
        }

        T w() const
        {
            return this->data()[3];
        }

        T &w()
        {
            return this->data()[3];
        }

        vector_t &operator=(const Eigen::Matrix<T, N, 1> &m)
        {
            for (size_t i = 0; i < N; i++)
            {
                this->data()[i] = m(i, 0);
            }

            return *this;
        }

        double length() const
        {
            return this->norm();
        }

    };
}

