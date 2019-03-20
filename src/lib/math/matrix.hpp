#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include "number.hpp"

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
            Eigen::Vector3d temp = Eigen::Vector3d::UnitX();

            switch (c)
            {
                case 'x':
                case 'X':
                    temp = Eigen::Vector3d::UnitX();
                    break;

                case 'y':
                case 'Y':
                    temp = Eigen::Vector3d::UnitY();
                    break;

                case 'z':
                case 'Z':
                    temp = Eigen::Vector3d::UnitZ();
                    break;

                default:
                    break;
            }
            Eigen::AngleAxisd rotv(deg2rad(deg), temp);
            set_R(rotv.toRotationMatrix());
        }

        transform_matrix rotationX(const double &deg)
        {
            *this = (*this)*transform_matrix(deg, 'x');
            return *this;
        }

        transform_matrix rotationY(const double &deg)
        {
            *this = (*this)*transform_matrix(deg, 'y');
            return *this;
        }

        transform_matrix rotationZ(const double &deg)
        {
            *this = (*this)*transform_matrix(deg, 'z');
            return *this;
        }

        transform_matrix translation(const double &x, const double &y, const double &z)
        {
            *this = (*this)*transform_matrix(x, y, z);
            return *this;
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

    class plane_matrix: public Eigen::Matrix3d
    {
    public:
        plane_matrix()
        {
            *this = Eigen::Matrix3d::Identity(3, 3);
        }

        plane_matrix(const Eigen::Matrix3d &m)
        {
            this->block<3, 3>(0, 0) = m;
        }

        plane_matrix(const double &x, const double &y)
        {
            *this = Eigen::Matrix3d::Identity(3, 3);
            set_p(Eigen::Vector2d(x, y));
        }

        plane_matrix(const double &deg)
        {
            *this = Eigen::Matrix3d::Identity(3, 3);
            set_R(rotation_mat(deg));
        }

        plane_matrix rotation(const double &deg)
        {
            *this = (*this)*plane_matrix(deg);
            return *this;
        }

        void set_p(const Eigen::Vector2d &p)
        {
            this->block<2, 1>(0, 2) = p;
        }

        void set_R(const Eigen::Matrix2d &r)
        {
            this->block<2, 2>(0, 0) = r;
        }

        plane_matrix &operator=(const Eigen::Matrix3d &m)
        {
            this->block<3, 3>(0, 0) = m;
            return *this;
        }
    private:
        Eigen::Matrix2d rotation_mat(const double &deg)
        {
            Eigen::Matrix2d temp;
            double rad = deg2rad(deg);
            temp<<std::cos(rad), std::sin(rad),
                  -std::sin(rad), std::cos(rad);
            return temp;
        }
    };
}
