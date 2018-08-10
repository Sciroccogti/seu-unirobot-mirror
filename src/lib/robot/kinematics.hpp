//
// Created by lcseu on 18-8-3.
//

#ifndef SEU_UNIROBOT_KINEMATICS_HPP
#define SEU_UNIROBOT_KINEMATICS_HPP

#include "humanoid.hpp"
#include "math/math.hpp"
#include "singleton.hpp"

namespace robot
{
    class kinematics: public singleton<kinematics>
    {
    public:
        kinematics()
        {
            D = ROBOT.get_bone_map()["hip"]->length_/2.0;
            A = ROBOT.get_bone_map()["rthigh"]->length_;
            B = ROBOT.get_bone_map()["rshank"]->length_;
            E = ROBOT.get_bone_map()["rfoot1"]->length_;
        }

        robot_math::transform_matrix leg_forward_kinematics(std::vector<double> &deg, const double &sign = 1.0)
        {
            if(deg.size()<6) return robot_math::transform_matrix();
            robot_math::transform_matrix T10, T21, T32, T43, T54, T65, T76, T_Mat;
            T10 = robot_math::transform_matrix(90, 'z')*robot_math::transform_matrix(180, 'x')*robot_math::transform_matrix(D, 0, 0);
            T21 = robot_math::transform_matrix(deg[0], 'z')*robot_math::transform_matrix(-90, 'x');
            T32 = robot_math::transform_matrix(deg[1]-90, 'z')*robot_math::transform_matrix(-90, 'x');
            T43 = robot_math::transform_matrix(deg[2], 'z')*robot_math::transform_matrix(A, 0, 0);
            T54 = robot_math::transform_matrix(deg[3], 'z')*robot_math::transform_matrix(B, 0, 0);
            T65 = robot_math::transform_matrix(deg[4], 'z')*robot_math::transform_matrix(90, 'x');
            T76 = robot_math::transform_matrix(deg[5], 'z')*robot_math::transform_matrix(-90, 'y')*robot_math::transform_matrix(0, 0, -E);
            robot_math::transform_matrix foot(0, sign*D, 0);
            T_Mat = T10*T21*T32*T43*T54*T65*T76;
            return robot_math::transform_matrix(foot*T_Mat.inverse());
        }

        bool leg_inverse_kinematics(const robot_math::transform_matrix &body,
                                const robot_math::transform_matrix &foot,
                                std::vector<double> &deg, const double &sign = 1.0)
        {
            Eigen::Vector3d tB = foot.p() + E*foot.a();
            Eigen::Vector3d r = foot.R().transpose()*(body.p()+body.R()*Eigen::Vector3d(0, sign*D, 0) - tB);
            double C = sqrt(r[0]*r[0]+r[1]*r[1]+r[2]*r[2]);
            if(C>A+B) return false;
            double c5 = (A*A+B*B-C*C)/(2*A*B);
            double q5t = acos(c5);
            double q5 = M_PI-q5t;

            double alpha = asin(A/C*sin(q5t));
            double q6 = -atan2(r[0], robot_math::sign(r[2])*sqrt(r[1]*r[1]+r[2]*r[2])) - alpha;

            double q7 = atan2(r[1], r[2]);
            if(q7>M_PI/2.0) q7 = q7-M_PI;
            else if(q7<-M_PI/2.0) q7 = q7+M_PI;

            Eigen::MatrixX3d R = body.R().transpose()*foot.R()*robot_math::RotX(robot_math::rad2deg(-q7))*robot_math::RotY(robot_math::rad2deg(-q6-q5));
            double q2 = atan2(-R(0,1), R(1,1));
            double cz = cos(q2), sz = sin(q2);
            double q3 = atan2(R(2,1), -R(0,1)*sz+R(1,1)*cz);
            double q4 = atan2(-R(2,0), R(2,2));

            deg.clear();
            deg.push_back(q2);
            deg.push_back(q3);
            deg.push_back(q4);
            deg.push_back(q5);
            deg.push_back(q6);
            deg.push_back(q7);
            return true;
        }

        double D, A, B, E;
    };

    #define RK kinematics::get_singleton()
}

#endif //SEU_UNIROBOT_KINEMATICS_HPP
