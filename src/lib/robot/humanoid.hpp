#pragma once

#include <string>
#include <mutex>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "robot_define.hpp"
#include "singleton.hpp"
#include "math/math.hpp"
#include "parser/robot_parser.hpp"
#include "parser/action_parser.hpp"
#include "parser/offset_parser.hpp"

namespace robot
{

    class humanoid: public singleton<humanoid>
    {
    public:
        bool arm_inverse_kinematics(const Eigen::Vector3d &hand, std::vector<double> &deg)
        {
            double x = hand[0];
            double z = hand[2] - (E_ + F_);
            double l = sqrt(x * x + z * z);

            if (l > E_ + F_)
            {
                return false;
            }

            double q3t = acos((E_ * E_ + F_ * F_ - l * l) / (2 * E_ * F_));
            double q3 = M_PI - q3t;
            double q1t0 = atan2(z, x);
            double q1t1;

            if (z <= 0)
            {
                q1t1 = -M_PI / 2.0 - q1t0;
            }
            else
            {
                if (x <= 0)
                {
                    q1t1 = 3.0 * M_PI / 2.0 - q1t0;
                }
                else
                {
                    q1t1 = -(M_PI / 2.0 + q1t0);
                }
            }

            double q1t2 = acos((E_ * E_ - F_ * F_ + l * l) / (2 * E_ * l));
            double q1 = (q1t1 + q1t2);
            deg.clear();
            deg.push_back(q1);
            deg.push_back(0.0);
            deg.push_back(q3);
            return true;
        }

        robot_math::transform_matrix leg_forward_kinematics(std::vector<double> &deg, const bool &left)
        {
            double sign = (left ? 1.0 : -1.0);

            if (deg.size() < 6)
            {
                return robot_math::transform_matrix();
            }

            robot_math::transform_matrix T10, T21, T32, T43, T54, T65, T76, T_Mat;
            T10 = robot_math::transform_matrix(90, 'z') * robot_math::transform_matrix(180, 'x') * robot_math::transform_matrix(D_ / 2.0, 0, 0);
            T21 = robot_math::transform_matrix(deg[0], 'z') * robot_math::transform_matrix(-90, 'x');
            T32 = robot_math::transform_matrix(deg[1] - 90, 'z') * robot_math::transform_matrix(-90, 'x');
            T43 = robot_math::transform_matrix(deg[2], 'z') * robot_math::transform_matrix(A_, 0, 0);
            T54 = robot_math::transform_matrix(deg[3], 'z') * robot_math::transform_matrix(B_, 0, 0);
            T65 = robot_math::transform_matrix(deg[4], 'z') * robot_math::transform_matrix(90, 'x');
            T76 = robot_math::transform_matrix(deg[5], 'z') * robot_math::transform_matrix(-90, 'y') * robot_math::transform_matrix(0, 0, -C_);
            robot_math::transform_matrix foot(0, sign * D_ / 2.0, 0);
            T_Mat = T10 * T21 * T32 * T43 * T54 * T65 * T76;
            return robot_math::transform_matrix(foot * T_Mat.inverse());
        }

        bool leg_inverse_kinematics(const robot_math::transform_matrix &body,
                                    const robot_math::transform_matrix &foot,
                                    std::vector<double> &deg, const bool &left = false)
        {
            double sign = (left ? 1.0 : -1.0);
            Eigen::Vector3d tB = foot.p() + C_ * foot.a();
            Eigen::Vector3d r = foot.R().transpose() * (body.p() + body.R() * Eigen::Vector3d(0, sign * D_ / 2.0, 0) - tB);
            double C = r.norm();

            if (C > A_ + B_)
            {
                return false;
            }

            double c5 = (A_ * A_ + B_ * B_ - C * C) / (2 * A_ * B_);
            double q5t = acos(c5);
            double q5 = M_PI - q5t;

            double alpha = asin(A_ / C * sin(q5t));
            double q6 = -atan2(r[0], robot_math::sign(r[2]) * sqrt(r[1] * r[1] + r[2] * r[2])) - alpha;

            double q7 = atan2(r[1], r[2]);

            if (q7 > M_PI / 2.0)
            {
                q7 = q7 - M_PI;
            }
            else if (q7 < -M_PI / 2.0)
            {
                q7 = q7 + M_PI;
            }

            Eigen::MatrixX3d R = body.R().transpose() * foot.R() * robot_math::RotX(robot_math::rad2deg(-q7)) * robot_math::RotY(robot_math::rad2deg(-q6 - q5));
            double q2 = atan2(-R(0, 1), R(1, 1));
            double cz = cos(q2), sz = sin(q2);
            double q3 = atan2(R(2, 1), -R(0, 1) * sz + R(1, 1) * cz);
            double q4 = atan2(-R(2, 0), R(2, 2));

            deg.clear();
            deg.push_back(q2);
            deg.push_back(q3);
            deg.push_back(q4);
            deg.push_back(q5);
            deg.push_back(q6);
            deg.push_back(q7);
            return true;
        }

        void init(const std::string &robot_file, const std::string &action_file, const std::string &offset_file)
        {
            main_bone_ = parser::robot_parser::parse(robot_file, bone_map_, joint_map_);
            parser::action_parser::parse(action_file, act_map_, pos_map_);
            parser::offset_parser::parse(offset_file, joint_map_);
            D_ = bone_map_["hip"]->length_;
            A_ = bone_map_["rthigh"]->length_;
            B_ = bone_map_["rshank"]->length_;
            C_ = bone_map_["rfoot1"]->length_;
            E_ = bone_map_["ruparm"]->length_;
            F_ = bone_map_["rlowarm"]->length_;
        }

        void set_degs(const std::map<int, float> &jdmap)
        {
            for (auto &j : jdmap)
            {
                get_joint(j.first)->set_deg(j.second);
            }
        }

        joint_ptr get_joint(const int &id)
        {
            for (auto &j : joint_map_)
                if (j.second->jid_ == id)
                {
                    return j.second;
                }

            throw class_exception<humanoid>("cannot find joint by id: " + std::to_string(id));
        }

        joint_ptr get_joint(const std::string &name)
        {
            for (auto &j : joint_map_)
                if (j.second->name_ == name)
                {
                    return j.second;
                }

            throw class_exception<humanoid>("cannot find joint by name: " + name);
        }

        inline double A() const
        {
            return A_;
        }
        inline double B() const
        {
            return B_;
        }
        inline double C() const
        {
            return C_;
        }
        inline double D() const
        {
            return D_;
        }
        inline double E() const
        {
            return E_;
        }
        inline double F() const
        {
            return F_;
        }
        inline double leg_length() const
        {
            return A_ + B_ + C_;
        }

        inline joint_map &get_joint_map()
        {
            return joint_map_;
        }
        inline joint_map get_joint_map() const
        {
            return joint_map_;
        }
        inline bone_map &get_bone_map()
        {
            return bone_map_;
        }
        inline bone_map get_bone_map() const
        {
            return bone_map_;
        }
        inline act_map &get_act_map()
        {
            return act_map_;
        }
        inline act_map get_act_map() const
        {
            return act_map_;
        }
        inline pos_map &get_pos_map()
        {
            return pos_map_;
        }
        inline pos_map get_pos_map() const
        {
            return pos_map_;
        }
        inline bone_ptr get_main_bone()
        {
            return main_bone_;
        }

    private:
        bone_ptr main_bone_;
        joint_map joint_map_;
        bone_map bone_map_;
        act_map act_map_;
        pos_map pos_map_;

        double D_, A_, B_, C_;
        double E_, F_;
    };

#define ROBOT humanoid::instance()
}
