#pragma once

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
    action_plan(const std::string &act_name, const bool &set_head = false)
        : plan("action_plan", "body"), act_name_(act_name), set_head_(set_head)
    {
        poses_.clear();
        pos_times_.clear();
    }

    action_plan(const std::vector< std::map<int, float> > &poses, const std::vector<int> &pos_times, const bool &set_head = true)
        : plan("action_plan", "body"), set_head_(set_head)
    {
        poses_ =  poses;
        pos_times_ = pos_times;
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

            std::string pos_name;
            std::map<std::string, float> jdegs;

            for (auto &p : aiter->second.poses)
            {
                act_t = p.act_time;
                pos_name = p.pos_name;
                jdegs.clear();
                jdegs = robot::ROBOT->get_pos_map()[pos_name].joints_deg;
                one_pos_deg.clear();

                for (auto &j : jdegs)
                {
                    one_pos_deg[robot::ROBOT->get_joint(j.first)->jid_] = j.second;
                }

                joints_plan jp(one_pos_deg, act_t, "body");

                if (!jp.perform())
                {
                    return false;
                }

                if (set_head_)
                {
                    joints_plan jp(one_pos_deg, act_t, "head");

                    if (!jp.perform())
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

                if (set_head_)
                {
                    joints_plan jp(one_pos_deg, act_t, "head");

                    if (!jp.perform())
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }
private:
    std::string act_name_;
    std::vector< std::map<int, float> > poses_;
    std::vector<int> pos_times_;
    bool set_head_;
};