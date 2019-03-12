#pragma once

#include <string>
#include <mutex>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "robot_define.hpp"
#include "singleton.hpp"
#include "math/math.hpp"

namespace robot
{

    class humanoid: public singleton<humanoid>
    {
    public:
        robot_math::transform_matrix get_foot_mat_from_pose(const robot_pose &pose, bool left);
        robot_math::transform_matrix get_body_mat_from_pose(const robot_pose &pose);

        bool arm_inverse_kinematics(const Eigen::Vector3d &hand, std::vector<double> &deg);

        robot_math::transform_matrix leg_forward_kinematics(std::vector<double> degs, bool left);
        bool leg_inverse_kinematics(const robot_math::transform_matrix &body, const robot_math::transform_matrix &foot,
                                    std::vector<double> &deg, const bool &left = false);
        bool leg_inverse_kinematics_walk(const robot_math::transform_matrix &body, const robot_math::transform_matrix &foot,
                                    std::vector<double> &deg, const bool &left = false);

        void ComputationForComAndFootpose(Eigen::Vector3d &Com, Eigen::Vector3d &footleft, Eigen::Vector3d &footright);

        void init(const std::string &robot_file, const std::string &action_file, const std::string &offset_file);

        void set_degs(const std::map<int, float> &jdmap);
        void set_real_degs(const std::map<int, float> &jdmap);

        std::vector<double> get_foot_degs(int support);
        std::vector<double> get_head_degs()
        {
            std::vector<double> res = {joint_map_["jhead1"]->get_deg(), joint_map_["jhead2"]->get_deg()};
            return res;
        }

        robot_math::transform_matrix body_mat;                   //init in walkEngine, used for ComputationForComAndFootpose 
        bool finished_one_step_flag;

        void print_joint_map();
        
        joint_ptr get_real_joint(const int &id);                 //for real joints
        joint_ptr get_real_joint(const std::string &name);       //for real joints
        
        struct{
            bool update_flag;    
            Eigen::Vector3d Com;                                 //center of mass
            Eigen::Vector3d leftfoot_pose;                       //the real pose in the end of step
            Eigen::Vector3d rightfoot_pose;

            Eigen::Vector3d leftfoot_pose_maxh;                  //the real pose when foot lift height is max
            Eigen::Vector3d rightfoot_pose_maxh;
        }conveyFeedbackParams;

        void empty_real_joint();
        void print_real_joint_map();
        void up_finished_one_step_flag();
        void down_finished_one_step_flag();

        joint_ptr get_joint(const int &id);
        joint_ptr get_joint(const std::string &name);

        inline double trunk_length()
        {
            return bone_map_["torso"]->length_;
        }

        inline double neck_length()
        {
            return bone_map_["head1"]->length_;
        }

        inline double head_length()
        {
            return bone_map_["camera1"]->length_;
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
        inline double leg_length_without_foot() const
        {
            return A_ + B_;
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

        inline joint_map &get_realJoint_map()
        {
            return real_joint_map_;
        }
        inline joint_map get_realJoint_map() const
        {
            return real_joint_map_;
        }

    private:
        bone_ptr main_bone_;
        joint_map joint_map_;
        joint_map real_joint_map_;

        bone_map bone_map_;
        act_map act_map_;
        pos_map pos_map_;

        double D_, A_, B_, C_;
        double E_, F_;

        enum Pose{
            hip,
            keen,
            ankle,
            leftHip,
            leftKeen,
            leftAnkle,
            rightHip,
            rightKeen,
            rightAnkle,
            num
        };
        robot_math::transform_matrix pose[num];
    };

#define ROBOT humanoid::instance()
}
