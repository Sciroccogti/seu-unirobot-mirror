#ifndef SEU_UNIROBOT_ACTUATOR_WALK_HPP
#define SEU_UNIROBOT_ACTUATOR_WALK_HPP

#include <map>
#include <thread>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include "motor/motor.hpp"
#include "singleton.hpp"

namespace walk
{
    class WalkEngine: public singleton<WalkEngine>
    {
    public:
        WalkEngine();
        ~WalkEngine();
        void start(sensor_ptr s);
        void stop() { is_alive_ = false; }
        void set_enable(const bool &e, const bool &run_action=false)
        {
            e_mutex_.lock();
            enable_ = e;
            run_action_ = run_action;
            //if(run_action) mode_ = MODE_ACT;
            if(e) mode_ = MODE_WALK;
            if(!enable_)
            {
                para_mutex_.lock();
                xt=0.0;
                yt=0.0;
                dt=0.0;
                para_mutex_.unlock();
            }
            e_mutex_.unlock();
        }
        void set_params(const Eigen::Vector4f &params);
    private:
        enum act_mode
        {
            MODE_WALK = 1,
            MODE_ACT = 2
        };

        act_mode mode_;
        void run();
        bool enable_;
        float x0, y0;
        float xt, yt, dt, ht;
        float supportfoot;
        float Cz;
        float wn;
        std::thread td_;
        bool is_alive_;
        bool run_action_;
        std::shared_ptr<motor> motor_;
        mutable std::mutex para_mutex_, e_mutex_;
        const float footYoffset = 0.02;
        const float T=0.5;
        const float td=0.05;
        const float x_range=0.04;
        const float y_range=0.02;
        const float d_range=20.0;
        const float h_max = 0.05;
    };

#define WALK WalkEngine::get_singleton()
}

#endif
