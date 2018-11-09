#include "humanoid.hpp"
#include "parser/robot_parser.hpp"
#include "parser/action_parser.hpp"
#include "parser/offset_parser.hpp"

namespace robot
{
    using namespace std;
    using namespace Eigen;
    using namespace robot_math;

    bool humanoid::arm_inverse_kinematics(const Vector3d &hand, vector<double> &deg)
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

    transform_matrix humanoid::leg_forward_kinematics(vector<double> &deg, const bool &left)
    {
        double sg = (left ? 1.0 : -1.0);

        if (deg.size() < 6)
        {
            return transform_matrix();
        }

        transform_matrix T10, T21, T32, T43, T54, T65, T76, T_Mat;
        T10 = transform_matrix(90, 'z') * transform_matrix(180, 'x') * transform_matrix(D_ / 2.0, 0, 0);
        T21 = transform_matrix(deg[0], 'z') * transform_matrix(-90, 'x');
        T32 = transform_matrix(deg[1] - 90, 'z') * transform_matrix(-90, 'x');
        T43 = transform_matrix(deg[2], 'z') * transform_matrix(A_, 0, 0);
        T54 = transform_matrix(deg[3], 'z') * transform_matrix(B_, 0, 0);
        T65 = transform_matrix(deg[4], 'z') * transform_matrix(90, 'x');
        T76 = transform_matrix(deg[5], 'z') * transform_matrix(-90, 'y') * transform_matrix(0, 0, -C_);
        transform_matrix foot(0, sg * D_ / 2.0, 0);
        T_Mat = T10 * T21 * T32 * T43 * T54 * T65 * T76;
        return transform_matrix(foot * T_Mat.inverse());
    }

    bool humanoid::leg_inverse_kinematics(const transform_matrix &body,
                                          const transform_matrix &foot, 
                                          vector<double> &deg, const bool &left)
    {
        double sg = (left ? 1.0 : -1.0);
        Vector3d p16 = foot.p() + C_ * foot.a();
        Vector3d p11 = body.p() + body.R()*Vector3d(0, sg*D_/2.0, 0);
        Vector3d r = foot.R().transpose() * (p11-p16);
        double Lr = r.norm();
        if (Lr > A_ + B_)
        {
            return false;
        }

        double alpha = acos((A_ * A_ + B_ * B_ - Lr * Lr) / (2 * A_ * B_));
        double q4 = M_PI - alpha;

        double beta = asin(A_ * sin(alpha) / Lr);
        double q5 = -atan2(r[0], sign(r[2]) * sqrt(r[1] * r[1] + r[2] * r[2])) - beta;

        double q6 = atan2(r[1], r[2]);

        if (q6 > M_PI / 2.0)
        {
            q6 = q6 - M_PI;
        }
        else if (q6 < -M_PI / 2.0)
        {
            q6 = q6 + M_PI;
        }

        MatrixX3d R = body.R().transpose() * foot.R() * RotX(-q6) *RotY(-q5)*RotY(-q4);
        double q1 = atan2(-R(0, 1), R(1, 1));
        double q3 = atan2(-R(2, 0), R(2, 2));
        double cz = cos(q1), sz = sin(q1);
        double q2 = atan2(R(2, 1), -R(0, 1) * sz + R(1, 1) * cz);

        deg.clear();
        deg.push_back(q1);
        deg.push_back(q2);
        deg.push_back(q3);
        deg.push_back(q4);
        deg.push_back(q5);
        deg.push_back(q6);
        return true;
    }

    void humanoid::init(const std::string &robot_file, const std::string &action_file,
                        const std::string &offset_file)
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


    void humanoid::set_degs(const std::map<int, float> &jdmap)
    {
        for (auto &j : jdmap)
        {
            get_joint(j.first)->set_deg(j.second);
        }
    }

    joint_ptr humanoid::get_joint(const int &id)
    {
        for (auto &j : joint_map_)
            if (j.second->jid_ == id)
            {
                return j.second;
            }

        throw class_exception<humanoid>("cannot find joint by id: " + std::to_string(id));
    }

    joint_ptr humanoid::get_joint(const std::string &name)
    {
        for (auto &j : joint_map_)
            if (j.second->name_ == name)
            {
                return j.second;
            }

        throw class_exception<humanoid>("cannot find joint by name: " + name);
    }
}