#ifndef SEU_UNIROBOT_ACTUATOR_LOOKAT_PLAN_HPP
#define SEU_UNIROBOT_ACTUATOR_LOOKAT_PLAN_HPP

#include "plan.hpp"
#include "logger.hpp"
#include "robot/humanoid.hpp"
#include "sensor/motor.hpp"
#include "class_exception.hpp"

class lookat_plan: public plan
{
public:
    lookat_plan(const float &yaw, const float &pitch, sensor_ptr s, const int &act_time=1)
        :plan("lookat_plan","head"), yaw_(yaw), pitch_(pitch), act_time_(act_time)
    {
        if(s == nullptr) throw class_exception<lookat_plan>("empty sensor ptr");
        motor_ = std::dynamic_pointer_cast<motor>(s);
    }
    
    bool perform()
    {
        std::map<int, float> latest_deg, jdmap, diff;
        latest_deg = motor_->get_head_degs();
        diff.clear();
        int pitch_id, yaw_id;
        yaw_id = robot::ROBOT.get_joint("jhead1")->jid_;
        pitch_id = robot::ROBOT.get_joint("jhead2")->jid_;
        
        diff[yaw_id] = yaw_-latest_deg[yaw_id];
        diff[pitch_id] = pitch_-latest_deg[pitch_id];
        float sdy = diff[yaw_id]/(float)act_time_;
        float sdp = diff[pitch_id]/(float)act_time_;
        
        for(int i=1;i<=act_time_;i++)
        {
            jdmap.clear();
            jdmap[yaw_id] = latest_deg[yaw_id]+i*sdy;
            jdmap[pitch_id] = latest_deg[pitch_id]+i*sdp;
            if(!motor_->add_head_degs(jdmap)) return false;
            while(!motor_->head_empty());
        }
        return true;
    }
private:
    float pitch_, yaw_, act_time_;
    std::shared_ptr<motor> motor_;
};

#endif