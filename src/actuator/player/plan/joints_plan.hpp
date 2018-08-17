#ifndef SEU_UNIROBOT_ACTUATOR_JOINTS_PLAN_HPP
#define SEU_UNIROBOT_ACTUATOR_JOINTS_PLAN_HPP

#include "plan.hpp"
#include "logger.hpp"
#include "robot/humanoid.hpp"
#include "sensor/motor.hpp"
#include "class_exception.hpp"
#include "math/math.hpp"

class joints_plan: public plan
{
public:
    joints_plan(const std::map<int, float> &jdmap, const int &act_time, sensor_ptr s)
            :plan("action_plan", "body")
    {
        if(s == nullptr) throw class_exception<action_plan>("empty sensor ptr");
        motor_ = std::dynamic_pointer_cast<motor>(s);
        poses_ =  poses;
        pos_times_ = pos_times;
    }

    bool set_joints(const std::map<int, float> &pos_deg, const int &act_t)
    {
        std::map<int, float> one_pos_deg, body_latest_deg, head_latest_deg, diffb, diffh, jdmap;
        one_pos_deg = pos_deg;
        body_latest_deg = motor_->get_body_degs();
        head_latest_deg = motor_->get_head_degs();
        int zero_n=0;
        diffb.clear();
        for(auto jdb:body_latest_deg)
        {
            diffb[jdb.first] = one_pos_deg[jdb.first] - body_latest_deg[jdb.first];
        }
        /*
        diffh.clear();
        for(auto jdb:head_latest_deg)
        {
            diffh[jdb.first] = one_pos_deg[jdb.first] - head_latest_deg[jdb.first];
            if(robot_math::is_zero(diffh[jdb.first])) zero_n++;
        }
        if(zero_n == robot::ROBOT.get_joint_map().size()) return;
         */
        for(int i=1;i<=act_t;i++)
        {
            jdmap.clear();
            for(auto db:diffb)
                jdmap[db.first] = body_latest_deg[db.first]+i*db.second/(float)act_t;
            if(!motor_->add_body_degs(jdmap)) return false;
            while (!motor_->body_empty());
            /*
            jdmap.clear();
            for(auto db:diffh)
                jdmap[db.first] = head_latest_deg[db.first]+i*db.second/(float)act_t;
            motor_->add_head_degs(jdmap);
             */
        }
        return true;
    }

    bool perform()
    {
        int act_t;
        std::map<int, float> one_pos_deg;
        if(poses_.empty()&&pos_times_.empty())
        {
            auto aiter = robot::ROBOT.get_act_map().find(act_name_);
            if(aiter == robot::ROBOT.get_act_map().end())
            {
                LOG(LOG_ERROR, "cannot find action: "+act_name_);
                return false;
            }
            std::string pos_name;
            std::map<std::string, float> jdegs;
            for(auto p:aiter->second.poses)
            {
                act_t = p.act_time;
                pos_name = p.pos_name;
                jdegs.clear();
                jdegs = robot::ROBOT.get_pos_map()[pos_name].joints_deg;
                one_pos_deg.clear();
                for(auto j:jdegs)
                    one_pos_deg[robot::ROBOT.get_joint(j.first)->jid_] = j.second;
                if(!set_joints(one_pos_deg, act_t)) return false;
            }
        }
        else
        {
            for(int i=0;i<poses_.size();i++)
            {
                act_t = pos_times_[i];
                one_pos_deg = poses_[i];
                if(!set_joints(one_pos_deg, act_t)) return false;
            }
        }
        return true;
    }
private:
    std::string act_name_;
    std::shared_ptr<motor> motor_;
    std::vector< std::map<int, float> > poses_;
    std::vector<int> pos_times_;
};

#endif