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
        void set_enable(const bool &e)
        {
            e_mutex_.lock();
            enable_ = e;
            e_mutex_.unlock();
        }
        void set_params(const Eigen::Vector4f &params);
    private:
        void run();
        bool enable_;
        float x0, y0;
        float xt, yt, dt, ht;
        float footYoffset = 0.02;
        const float T=0.5;
        const float td=0.05;
        float supportfoot;
        float Cz;
        float wn;
        std::thread td_;
        bool is_alive_;
        std::shared_ptr<motor> motor_;
        mutable std::mutex para_mutex_, e_mutex_;
        const float x_range=0.04;
        const float y_range=0.02;
        const float d_range=20.0;
        const float h_max = 0.05;
    };

#define WALK WalkEngine::get_singleton()
}

#endif
