#pragma once

#include <algorithm>
#include <cmath>
#include "plan.hpp"
#include "robot/humanoid.hpp"
#include "class_exception.hpp"
#include "math/math.hpp"
#include "joints_plan.hpp"
#include "lookat_plan.hpp"
#include "walk/WalkEngine.hpp"
#include "core/adapter.hpp"

class action_plan: public plan
{
public:
    action_plan(const std::string &act_name)
        : plan("action_plan", "body"), act_name_(act_name)
    {
        poses_.clear();
        pos_times_.clear();
        step_ = 0.01;

        auto aiter = robot::ROBOT->get_act_map().find(act_name_);
        if (aiter == robot::ROBOT->get_act_map().end())
        {
            std::cout << "cannot find action: " + act_name_ << "\n";
        }
        else
        {
            std::vector<robot::robot_one_pos> poses = aiter->second.poses;
            for(int i=0;i<poses.size();i++)
            {
                pos_times_.push_back(poses[i].act_time);
                poses_.push_back(robot::ROBOT->get_pos_map()[poses[i].pos_name].pose_info);
            }
        }
    }

    action_plan(const std::vector< std::map<robot::robot_motion, robot::robot_pose> > &poses, const std::vector<int> &pos_times)
        : plan("action_plan", "body")
    {
        poses_ =  poses;
        pos_times_ = pos_times;
        step_ = 0.01;
    }

    std::vector< std::map<robot::robot_motion, robot::robot_pose> > get_poses(robot::robot_pos &pos1, robot::robot_pos &pos2, int act_time)
    {
        Eigen::Vector3d poseb1(pos1.pose_info[robot::MOTION_BODY].x, pos1.pose_info[robot::MOTION_BODY].y,
                               pos1.pose_info[robot::MOTION_BODY].z);
        Eigen::Vector3d poseb2(pos2.pose_info[robot::MOTION_BODY].x, pos2.pose_info[robot::MOTION_BODY].y,
                               pos2.pose_info[robot::MOTION_BODY].z);
        Eigen::Vector3d posel1(pos1.pose_info[robot::MOTION_LEFT_FOOT].x, pos1.pose_info[robot::MOTION_LEFT_FOOT].y,
                               pos1.pose_info[robot::MOTION_LEFT_FOOT].z);
        Eigen::Vector3d posel2(pos2.pose_info[robot::MOTION_LEFT_FOOT].x, pos2.pose_info[robot::MOTION_LEFT_FOOT].y,
                               pos2.pose_info[robot::MOTION_LEFT_FOOT].z);
        Eigen::Vector3d poser1(pos1.pose_info[robot::MOTION_RIGHT_FOOT].x, pos1.pose_info[robot::MOTION_RIGHT_FOOT].y,
                               pos1.pose_info[robot::MOTION_RIGHT_FOOT].z);
        Eigen::Vector3d poser2(pos2.pose_info[robot::MOTION_RIGHT_FOOT].x, pos2.pose_info[robot::MOTION_RIGHT_FOOT].y,
                               pos2.pose_info[robot::MOTION_RIGHT_FOOT].z);
        Eigen::Vector3d pposeb1(pos1.pose_info[robot::MOTION_BODY].pitch, pos1.pose_info[robot::MOTION_BODY].roll,
                                pos1.pose_info[robot::MOTION_BODY].yaw);
        Eigen::Vector3d pposeb2(pos2.pose_info[robot::MOTION_BODY].pitch, pos2.pose_info[robot::MOTION_BODY].roll,
                                pos2.pose_info[robot::MOTION_BODY].yaw);
        Eigen::Vector3d pposel1(pos1.pose_info[robot::MOTION_LEFT_FOOT].pitch, pos1.pose_info[robot::MOTION_LEFT_FOOT].roll,
                                pos1.pose_info[robot::MOTION_LEFT_FOOT].yaw);
        Eigen::Vector3d pposel2(pos2.pose_info[robot::MOTION_LEFT_FOOT].pitch, pos2.pose_info[robot::MOTION_LEFT_FOOT].roll,
                                pos2.pose_info[robot::MOTION_LEFT_FOOT].yaw);
        Eigen::Vector3d pposer1(pos1.pose_info[robot::MOTION_RIGHT_FOOT].pitch, pos1.pose_info[robot::MOTION_RIGHT_FOOT].roll,
                                pos1.pose_info[robot::MOTION_RIGHT_FOOT].yaw);
        Eigen::Vector3d pposer2(pos2.pose_info[robot::MOTION_RIGHT_FOOT].pitch, pos2.pose_info[robot::MOTION_RIGHT_FOOT].roll,
                                pos2.pose_info[robot::MOTION_RIGHT_FOOT].yaw);

        Eigen::Vector3d poselh1(pos1.pose_info[robot::MOTION_LEFT_HAND].x, pos1.pose_info[robot::MOTION_LEFT_HAND].y,
                                pos1.pose_info[robot::MOTION_LEFT_HAND].z);
        Eigen::Vector3d poselh2(pos2.pose_info[robot::MOTION_LEFT_HAND].x, pos2.pose_info[robot::MOTION_LEFT_HAND].y,
                                pos2.pose_info[robot::MOTION_LEFT_HAND].z);
        Eigen::Vector3d poserh1(pos1.pose_info[robot::MOTION_RIGHT_HAND].x, pos1.pose_info[robot::MOTION_RIGHT_HAND].y,
                                pos1.pose_info[robot::MOTION_RIGHT_HAND].z);
        Eigen::Vector3d poserh2(pos2.pose_info[robot::MOTION_RIGHT_HAND].x, pos2.pose_info[robot::MOTION_RIGHT_HAND].y,
                                pos2.pose_info[robot::MOTION_RIGHT_HAND].z);

        Eigen::Vector3d dposeb = poseb2-poseb1;
        Eigen::Vector3d dposel = posel2-posel1;
        Eigen::Vector3d dposer = poser2-poser1;
        Eigen::Vector3d dposelh = poselh2-poselh1;
        Eigen::Vector3d dposerh = poserh2-poserh1;

        Eigen::Vector3d dpposeb = pposeb2-pposeb1;
        Eigen::Vector3d dpposel = pposel2-pposel1;
        Eigen::Vector3d dpposer = pposer2-pposer1;
        double maxdb = std::max(std::max(std::abs(dposeb.x()), std::abs(dposeb.y())), std::abs(dposeb.z()));
        double maxdl = std::max(std::max(std::abs(dposel.x()), std::abs(dposel.y())), std::abs(dposel.z()));
        double maxdr = std::max(std::max(std::abs(dposel.x()), std::abs(dposel.y())), std::abs(dposel.z()));
        double maxv = std::max(std::max(maxdb, maxdl), maxdr);
        std::vector< std::map<robot::robot_motion, robot::robot_pose> > act_poses;

        int count;
        if(robot_math::is_zero(maxv-maxdb))
        {
            if(robot_math::is_zero(maxdb-std::abs(dposeb.x())))
            {
                count = dposeb.x()/step_;
            }
            else if(robot_math::is_zero(maxdb-std::abs(dposeb.y())))
            {
                count = dposeb.y()/step_;
            }
            else
            {
                count = dposeb.z()/step_;
            }
        }
        else if(robot_math::is_zero(maxv-maxdl))
        {
            if(robot_math::is_zero(maxdl-std::abs(dposel.x())))
            {
                count = dposel.x()/step_;
            }
            else if(robot_math::is_zero(maxdl-std::abs(dposel.y())))
            {
                count = dposel.y()/step_;
            }
            else
            {
                count = dposel.z()/step_;
            }
        }
        else
        {
            if(robot_math::is_zero(maxdr-std::abs(dposer.x())))
            {
                count = dposer.x()/step_;
            }
            else if(robot_math::is_zero(maxdr-std::abs(dposer.y())))
            {
                count = dposer.y()/step_;
            }
            else
            {
                count = dposer.z()/step_;
            }
        }
        if(act_time>count) count = act_time;
        double dbx = dposeb.x()/count, dby = dposeb.y()/count, dbz = dposeb.z()/count;
        double dbpi = dpposeb.x()/count, dbro = dpposeb.y()/count, dbya = dpposeb.z()/count;
        double dlx = dposel.x()/count, dly = dposel.y()/count, dlz = dposel.z()/count;
        double dlpi = dpposel.x()/count, dlro = dpposel.y()/count, dlya = dpposel.z()/count;
        double drx = dposer.x()/count, dry = dposer.y()/count, drz = dposer.z()/count;
        double drpi = dpposer.x()/count, drro = dpposer.y()/count, drya = dpposer.z()/count;

        double dlhx = dposelh.x()/count, dlhz = dposelh.z()/count;
        double drhx = dposerh.x()/count, drhz = dposerh.z()/count;

        for(int i=0;i<count;i++)
        {
            std::map<robot::robot_motion, robot::robot_pose> temp_pose_map;
            robot::robot_pose temp_pose;
            temp_pose.x = poselh1.x()+i*dlhx;
            temp_pose.z = poselh1.z()+i*dlhz;
            temp_pose_map[robot::MOTION_LEFT_HAND] = temp_pose;
            temp_pose.x = poserh1.x()+i*drhx;
            temp_pose.z = poserh1.z()+i*drhz;
            temp_pose_map[robot::MOTION_RIGHT_HAND] = temp_pose;
            temp_pose.x = poseb1.x()+i*dbx;
            temp_pose.y = poseb1.y()+i*dby;
            temp_pose.z = poseb1.z()+i*dbz;
            temp_pose.pitch = pposeb1.x()+i*dbpi;
            temp_pose.roll = pposeb1.y()+i*dbro;
            temp_pose.yaw = pposeb1.z()+i*dbya;
            temp_pose_map[robot::MOTION_BODY] = temp_pose;
            temp_pose.x = posel1.x()+i*dlx;
            temp_pose.y = posel1.y()+i*dly;
            temp_pose.z = posel1.z()+i*dlz;
            temp_pose.pitch = pposel1.x()+i*dlpi;
            temp_pose.roll = pposel1.y()+i*dlro;
            temp_pose.yaw = pposel1.z()+i*dlya;
            temp_pose_map[robot::MOTION_LEFT_FOOT] = temp_pose;
            temp_pose.x = poser1.x()+i*drx;
            temp_pose.y = poser1.y()+i*dry;
            temp_pose.z = poser1.z()+i*drz;
            temp_pose.pitch = pposer1.x()+i*drpi;
            temp_pose.roll = pposer1.y()+i*drro;
            temp_pose.yaw = pposer1.z()+i*drya;
            temp_pose_map[robot::MOTION_RIGHT_FOOT] = temp_pose;
            act_poses.push_back(temp_pose_map);
        }
        return act_poses;
    }

    bool get_degs(const robot_math::transform_matrix &body_mat, const robot_math::transform_matrix &leftfoot_mat,
            const robot_math::transform_matrix &rightfoot_mat, const Eigen::Vector3d &lefthand, const Eigen::Vector3d &righthand,
                  std::map<int, float> &degs_map)
    {
        std::vector<double> degs;
        degs_map.clear();
        if (robot::ROBOT->leg_inverse_kinematics(body_mat, leftfoot_mat, degs, true))
        {
            degs_map[robot::ROBOT->get_joint("jlhip3")->jid_] = robot_math::rad2deg(degs[0]);
            degs_map[robot::ROBOT->get_joint("jlhip2")->jid_] = robot_math::rad2deg(degs[1]);
            degs_map[robot::ROBOT->get_joint("jlhip1")->jid_] = robot_math::rad2deg(degs[2]);
            degs_map[robot::ROBOT->get_joint("jlknee")->jid_] = robot_math::rad2deg(degs[3]);
            degs_map[robot::ROBOT->get_joint("jlankle2")->jid_] = robot_math::rad2deg(degs[4]);
            degs_map[robot::ROBOT->get_joint("jlankle1")->jid_] = robot_math::rad2deg(degs[5]);
        }
        else
        {
            return false;
        }

        if (robot::ROBOT->leg_inverse_kinematics(body_mat, rightfoot_mat, degs, false))
        {
            degs_map[robot::ROBOT->get_joint("jrhip3")->jid_] = robot_math::rad2deg(degs[0]);
            degs_map[robot::ROBOT->get_joint("jrhip2")->jid_] = robot_math::rad2deg(degs[1]);
            degs_map[robot::ROBOT->get_joint("jrhip1")->jid_] = robot_math::rad2deg(degs[2]);
            degs_map[robot::ROBOT->get_joint("jrknee")->jid_] = robot_math::rad2deg(degs[3]);
            degs_map[robot::ROBOT->get_joint("jrankle2")->jid_] = robot_math::rad2deg(degs[4]);
            degs_map[robot::ROBOT->get_joint("jrankle1")->jid_] = robot_math::rad2deg(degs[5]);
        }
        else
        {
            return false;
        }
        if (robot::ROBOT->arm_inverse_kinematics(lefthand, degs))
        {
            degs_map[robot::ROBOT->get_joint("jlshoulder1")->jid_] = robot_math::rad2deg(degs[0]);
            degs_map[robot::ROBOT->get_joint("jlelbow")->jid_] = -robot_math::rad2deg(degs[2]);
        }
        else
        {
            return false;
        }
        if (robot::ROBOT->arm_inverse_kinematics(righthand, degs))
        {
            degs_map[robot::ROBOT->get_joint("jrshoulder1")->jid_] = robot_math::rad2deg(degs[0]);
            degs_map[robot::ROBOT->get_joint("jrelbow")->jid_] = robot_math::rad2deg(degs[2]);
        }
        else
        {
            return false;
        }
        return true;
    }

    bool perform()
    {
        if (MADT->mode() == adapter::MODE_WALK)
        {
            MADT->mode() = adapter::MODE_READY;
        }
        else if (MADT->mode() == adapter::MODE_NONE)
        {
            MADT->mode() = adapter::MODE_ACT;
        }

        while (MADT->mode() != adapter::MODE_ACT);

        int act_t;
        std::map<int, float> one_pos_deg;

        if (poses_.empty() && pos_times_.empty())
        {
            auto aiter = robot::ROBOT->get_act_map().find(act_name_);

            if (aiter == robot::ROBOT->get_act_map().end())
            {
                std::cout << "cannot find action: " + act_name_ << "\n";
                return false;
            }

            std::map<std::string, float> jdegs;
            std::string pos_name1, pos_name2;
            robot::robot_pos pos1, pos2;

            robot_math::transform_matrix body_mat, leftfoot_mat, rightfoot_mat;
            Eigen::Vector3d lefthand, righthand;
            std::vector<robot::robot_one_pos> poses = aiter->second.poses;
            pos_name1 = poses[0].pos_name;
            act_t = poses[0].act_time;
            pos1 = robot::ROBOT->get_pos_map()[pos_name1];

            body_mat = robot::ROBOT->get_body_mat_from_pose(pos1.pose_info[robot::MOTION_BODY]);
            leftfoot_mat = robot::ROBOT->get_foot_mat_from_pose(pos1.pose_info[robot::MOTION_LEFT_FOOT], true);
            rightfoot_mat = robot::ROBOT->get_foot_mat_from_pose(pos1.pose_info[robot::MOTION_RIGHT_FOOT], false);
            righthand[0] = pos1.pose_info[robot::MOTION_RIGHT_HAND].x;
            righthand[2] = pos1.pose_info[robot::MOTION_RIGHT_HAND].z;
            lefthand[0] = pos1.pose_info[robot::MOTION_LEFT_HAND].x;
            lefthand[2] = pos1.pose_info[robot::MOTION_LEFT_HAND].z;
            if(get_degs(body_mat, leftfoot_mat, rightfoot_mat, lefthand, righthand, one_pos_deg))
            {
                joints_plan jp(one_pos_deg, act_t, "body");
                if (!jp.perform())
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
            if(poses.size()<2) return true;
            for(int i=1;i<poses.size();i++)
            {
                pos_name1 = poses[i-1].pos_name;
                pos_name2 = poses[i].pos_name;
                pos1 = robot::ROBOT->get_pos_map()[pos_name1];
                pos2 = robot::ROBOT->get_pos_map()[pos_name2];
                std::vector< std::map<robot::robot_motion, robot::robot_pose> > act_poses;
                act_poses = get_poses(pos1, pos2, poses[i].act_time);
                for(auto act_pos:act_poses)
                {
                    body_mat = robot::ROBOT->get_body_mat_from_pose(act_pos[robot::MOTION_BODY]);
                    leftfoot_mat = robot::ROBOT->get_foot_mat_from_pose(act_pos[robot::MOTION_LEFT_FOOT], true);
                    rightfoot_mat = robot::ROBOT->get_foot_mat_from_pose(act_pos[robot::MOTION_RIGHT_FOOT], false);
                    righthand[0] = act_pos[robot::MOTION_RIGHT_HAND].x;
                    righthand[2] = act_pos[robot::MOTION_RIGHT_HAND].z;
                    lefthand[0] = act_pos[robot::MOTION_LEFT_HAND].x;
                    lefthand[2] = act_pos[robot::MOTION_LEFT_HAND].z;
                    if(get_degs(body_mat, leftfoot_mat, rightfoot_mat, lefthand, righthand, one_pos_deg))
                    {
                        joints_plan jp(one_pos_deg, 1, "body");
                        if (!jp.perform())
                        {
                            return false;
                        }
                    }
                    else
                    {
                        return false;
                    }
                }
            }
        }
        else
        {
            for (size_t i = 0; i < poses_.size(); i++)
            {
                act_t = pos_times_[i];
                one_pos_deg = poses_[i];
                joints_plan jp(one_pos_deg, act_t, "body");

                if (!jp.perform())
                {
                    return false;
                }
            }
        }

        return true;
    }
private:
    std::string act_name_;
    std::vector< std::map<robot::robot_motion, robot::robot_pose> > poses_;
    std::vector<int> pos_times_;
    double step_;
};
