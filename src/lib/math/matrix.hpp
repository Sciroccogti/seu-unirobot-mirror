#ifndef SEU_UNIROBOT_MATRIX_HPP
#define SEU_UNIROBOT_MATRIX_HPP

#include <cmath>
#include <eigen3/Eigen/Dense>
#include "math_common.hpp"

namespace robot_math
{
    class transform_matrix: public Eigen::Matrix4d
    {
    public:
        transform_matrix()
        {
            *this = Eigen::Matrix4d::Identity(4, 4);
        }

        transform_matrix(const Eigen::Matrix4d &m)
        {
            this->block<4, 4>(0, 0) = m;
        }

        transform_matrix(const double &x, const double &y, const double &z)
        {
            *this = Eigen::Matrix4d::Identity(4, 4);
            set_p(Eigen::Vector3d(x, y, z));
        }

        transform_matrix(const double &deg, const char &c = 'x')
        {
            *this = Eigen::Matrix4d::Identity(4, 4);

            switch (c)
            {
                case 'x':
                case 'X':
                    set_R(RotX(deg));
                    break;

                case 'y':
                case 'Y':
                    set_R(RotY(deg));
                    break;

                case 'z':
                case 'Z':
                    set_R(RotZ(deg));
                    break;

                default:
                    break;
            }
        }

        Eigen::Matrix3d R() const
        {
            return this->block<3, 3>(0, 0);
        }

        Eigen::Vector3d p() const
        {
            return this->block<3, 1>(0, 3);
        }

        Eigen::Vector3d n() const
        {
            return this->block<3, 1>(0, 0);
        }

        Eigen::Vector3d o() const
        {
            return this->block<3, 1>(0, 1);
        }

        Eigen::Vector3d a() const
        {
            return this->block<3, 1>(0, 2);
        }

        void set_p(const Eigen::Vector3d &p)
        {
            this->block<3, 1>(0, 3) = p;
        }

        void set_R(const Eigen::Matrix3d &r)
        {
            this->block<3, 3>(0, 0) = r;
        }

        transform_matrix &operator=(const Eigen::Matrix4d &m)
        {
            this->block<4, 4>(0, 0) = m;
            return *this;
        }
    };
}

#endif